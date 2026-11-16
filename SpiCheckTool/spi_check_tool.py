# -*- coding: utf-8 -*-
"""
SPI 通信验证上位机
------------------------------------------------------------------
配套固件：OsBurnFinalTest_G8U6（STM32G031G8U6，Modbus RTU 从站 9600-8-N-1）

固件侧协议（不改固件，完全按现有 Parse() 的分支走）：
  写保持寄存器 40001 起  -> recvBuf[] -> Parse() 按 recvBuf[0] 分发
  读输入寄存器 30001 起  -> sendBuf[]（命令执行结果）

  本程序只用到两族命令：
    cmdType = 1（系统）
        recvBuf = [1, 0x104]              -> 读固件版本，out[0],out[1] = 主.次
        recvBuf = [1, 0x207, mV]          -> 设 VDD（MCP4725，单位 mV）
    cmdType = 6（SPI 透传）
        recvBuf = [6, 0, len, b0..bn-1]   -> 在 MOSI 上发 len 个字节
        读 30001 起 len 个                -> MISO 收回来的 len 个字节，原样返回

  固件里是拿 usAddress 直接和 40001 / 30001 比较的，但 FreeModbus 在交给回调前
  会先 usRegAddress++，所以 PDU 里要发 40000(0x9C40) / 30000(0x7530)。

引脚：SCK=PA6(EX_SCL)  MOSI=PC6  MISO=PA8  CS=PB1
"""

import os
import sys
import time
import queue
import threading
import datetime
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from tkinter.scrolledtext import ScrolledText

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # 打包后会带进来，这里只是源码直跑时的提示
    serial = None

APP_TITLE = "SPI 通信验证上位机  |  OsBurnFinalTest_G8U6"
APP_VERSION = "1.0"

# ----------------------------------------------------------------------------
# 协议常量
# ----------------------------------------------------------------------------
# 固件里比较的是 30001 / 40001（app_freertos.c 的 REG_INPUT_START / REG_HOLDING_START），
# 但 FreeModbus 在把地址交给回调前会先 +1：
#     mbfuncholding.c:123   usRegAddress++;
#     mbfuncinput.c:72      usRegAddress++;
# 所以 PDU 里必须发 40000 / 30000，多 1 会被判成 40002 -> 越界 -> 异常码 0x02。
ADDR_HOLDING = 40000        # PDU 地址（固件回调收到的是 40001）
ADDR_INPUT = 30000          # PDU 地址（固件回调收到的是 30001）

CMD_TYPE_SYS = 1
CMD_TYPE_SPI = 6

SYS_GET_VERSION = 0x104
SYS_SET_VDD = 0x207
SYS_SET_POWER = 0x208      # 电源开关（需要固件补丁，见 README）
SYS_2200_FT = 0x206        # 老命令：跑一遍完整终测，能间接验证 SPI

SPI_MAX_BYTES = 16          # 固件 ParseSPICmd 里 tx[16] / rx[16]
MODBUS_MAX_REGS = 64        # 固件 RECVBUF_LENS

# 关注的气压/温度寄存器（WF 系列 4 线 SPI 读寄存器：发 80 <reg> FF，收 rx[2]）
REG_STATUS = 0x02
REG_P_MSB, REG_P_MID, REG_P_LSB = 0x06, 0x07, 0x08
REG_T_MSB, REG_T_LSB = 0x09, 0x0A

FONT_UI = ("Microsoft YaHei UI", 9)
FONT_MONO = ("Consolas", 10)


# ----------------------------------------------------------------------------
# 字节工具
# ----------------------------------------------------------------------------
def crc16(data: bytes) -> int:
    """Modbus RTU CRC16（多项式 0xA001，初值 0xFFFF）"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def add_crc(body: bytes) -> bytes:
    c = crc16(body)
    return bytes(body) + bytes((c & 0xFF, (c >> 8) & 0xFF))


def to_hex(data) -> str:
    return " ".join("%02X" % b for b in data)


def parse_hex(text: str) -> bytes:
    """宽松解析：支持 80 06 FF / 80,06,FF / 0x80 0x06 0xFF"""
    s = text.replace(",", " ").replace(";", " ")
    s = s.replace("0x", " ").replace("0X", " ")
    out = []
    for tok in s.split():
        if len(tok) == 1:
            tok = "0" + tok
        if len(tok) % 2:
            raise ValueError("字节长度不对：%s" % tok)
        for i in range(0, len(tok), 2):
            out.append(int(tok[i:i + 2], 16))
    return bytes(out)


def press_kpa(raw24: int) -> float:
    """24 位气压码值 -> kPa。公式照抄 资料/WF5803_1BAR_IIC驱动包/wfsensor.c 的 calculatePress()"""
    if raw24 >= 8388608:
        f = (raw24 - 16777216) / 8388608.0
    else:
        f = raw24 / 8388608.0
    return f * 125.0 + 17.5


def temp_c(raw16: int) -> float:
    """16 位温度码值 -> ℃。同样照抄 wfsensor.c（>32768 走负温分支）"""
    if raw16 > 32768:
        return (raw16 - 65844) / 256.0
    return (raw16 - 308) / 256.0


def fmt_reading(p: int, t: int) -> str:
    """摘要行。全 00 / 全 FF 是无效值，公式算出来会得到 17.5 kPa 这种误导性结果，所以单独标出来"""
    if p in (0, 0xFFFFFF) or t in (0, 0xFFFF):
        return "气压 0x%06X（无效）    温度 0x%04X（无效）" % (p, t)
    return "气压 0x%06X ≈ %.2f kPa    温度 0x%04X ≈ %.1f ℃" % (p, press_kpa(p), t, temp_c(t))


# ----------------------------------------------------------------------------
# Modbus RTU 主站（只做 0x10 / 0x04 两条功能码，够本固件用）
# ----------------------------------------------------------------------------
class ModbusError(Exception):
    pass


class ModbusRtuMaster:
    def __init__(self):
        self.ser = None
        self.slave = 1
        self.timeout = 1.0
        self._lock = threading.RLock()

    @property
    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def open(self, port, baud=9600, slave=1, timeout=1.0):
        self.close()
        self.ser = serial.Serial(
            port=port, baudrate=baud, bytesize=8,
            parity=serial.PARITY_NONE, stopbits=1,
            timeout=0.02, write_timeout=1.0,
        )
        self.slave = int(slave)
        self.timeout = float(timeout)

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _read_exact(self, n, deadline):
        buf = b""
        while len(buf) < n and time.time() < deadline:
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf += chunk
        return buf

    @staticmethod
    def _check_crc(buf):
        c = crc16(buf[:-2])
        if bytes((c & 0xFF, (c >> 8) & 0xFF)) != buf[-2:]:
            raise ModbusError("CRC 校验失败，可能是波特率/校验位不对或线路干扰")

    def _transact(self, body: bytes, timeout: float = None) -> bytes:
        """按功能码分段收，异常应答只有 5 字节，不能按固定长度干等"""
        if not self.is_open:
            raise ModbusError("串口未连接")
        to = self.timeout if timeout is None else float(timeout)
        frame = add_crc(body)
        with self._lock:
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            self.ser.write(frame)
            deadline = time.time() + to

            hdr = self._read_exact(2, deadline)
            if not hdr:
                raise ModbusError("从站无应答：检查串口、波特率、从站地址、接线")
            if len(hdr) < 2:
                raise ModbusError("应答不完整：只收到 %d 字节" % len(hdr))
            if hdr[0] != self.slave:
                raise ModbusError("从站地址不匹配：应答 0x%02X，期望 0x%02X" % (hdr[0], self.slave))

            func = hdr[1]
            if func & 0x80:
                buf = hdr + self._read_exact(3, deadline)
                if len(buf) < 5:
                    raise ModbusError("异常应答不完整")
                self._check_crc(buf)
                exc = buf[2]
                hint = ""
                if exc == 0x02:
                    hint = "（地址不对：固件比较的是 40001/30001，FreeModbus 会先 +1，所以 PDU 要发 40000/30000）"
                elif exc == 0x03:
                    hint = "（寄存器数量超出固件缓冲）"
                raise ModbusError("从站返回异常码 0x%02X%s" % (exc, hint))

            if func != body[1]:
                raise ModbusError("功能码不匹配：应答 0x%02X，期望 0x%02X" % (func, body[1]))

            if func == 0x04:
                bc = self._read_exact(1, deadline)
                if not bc:
                    raise ModbusError("应答不完整：缺字节数")
                buf = hdr + bc + self._read_exact(bc[0] + 2, deadline)
                if len(buf) < 5 + bc[0]:
                    raise ModbusError("应答不完整：收到 %d 字节" % len(buf))
            else:
                buf = hdr + self._read_exact(6, deadline)
                if len(buf) < 8:
                    raise ModbusError("应答不完整：收到 %d 字节" % len(buf))

        self._check_crc(buf)
        return buf

    def write_holding(self, values, addr=ADDR_HOLDING, timeout=None):
        n = len(values)
        if n == 0:
            raise ModbusError("空写入")
        if n > MODBUS_MAX_REGS:
            raise ModbusError("一次最多写 %d 个寄存器" % MODBUS_MAX_REGS)
        body = bytearray((self.slave, 0x10, (addr >> 8) & 0xFF, addr & 0xFF,
                          (n >> 8) & 0xFF, n & 0xFF, n * 2))
        for v in values:
            v = int(v) & 0xFFFF
            body += bytes(((v >> 8) & 0xFF, v & 0xFF))
        self._transact(bytes(body), timeout)

    def read_input(self, qty, addr=ADDR_INPUT, timeout=None):
        if qty <= 0:
            return []
        body = bytes((self.slave, 0x04, (addr >> 8) & 0xFF, addr & 0xFF,
                      (qty >> 8) & 0xFF, qty & 0xFF))
        buf = self._transact(body, timeout)
        if buf[2] != qty * 2:
            raise ModbusError("字节数不匹配：%d != %d" % (buf[2], qty * 2))
        return [(buf[3 + 2 * i] << 8) | buf[4 + 2 * i] for i in range(qty)]


# ----------------------------------------------------------------------------
# 板卡业务封装
# ----------------------------------------------------------------------------
class Board:
    def __init__(self, mb: ModbusRtuMaster):
        self.mb = mb
        self.last_vdd = 3300

    def spi(self, data: bytes, timeout=None) -> bytes:
        n = len(data)
        if n == 0:
            raise ValueError("至少发 1 个字节")
        if n > SPI_MAX_BYTES:
            raise ValueError("单笔最多 %d 字节（固件缓冲 tx[16]）" % SPI_MAX_BYTES)
        self.mb.write_holding([CMD_TYPE_SPI, 0x00, n] + list(data), timeout=timeout)
        regs = self.mb.read_input(n, timeout=timeout)
        return bytes(r & 0xFF for r in regs)

    def spi_read_reg(self, reg: int, timeout=None) -> int:
        rx = self.spi(bytes((0x80, reg & 0xFF, 0xFF)), timeout=timeout)
        return rx[2]

    def spi_write_reg(self, reg: int, value: int, timeout=None):
        self.spi(bytes((0x00, reg & 0xFF, value & 0xFF)), timeout=timeout)

    def spi_mode_4wire(self, timeout=None):
        self.spi(bytes((0x00, 0x00, 0x01)), timeout=timeout)

    def get_version(self):
        self.mb.write_holding([CMD_TYPE_SYS, SYS_GET_VERSION])
        r = self.mb.read_input(2)
        return r[0], r[1]

    def set_vdd(self, mv: int):
        mv = int(mv)
        if mv < 0 or mv > 65535:
            raise ValueError("VDD 必须是 0~65535 mV")
        self.mb.write_holding([CMD_TYPE_SYS, SYS_SET_VDD, mv], timeout=2.0)
        self.last_vdd = mv

    def set_power(self, vdd_on, vpp_on=False, os_on=False):
        """电源开关（固件 0x208）。返回 out[0]：1 = 固件支持并已执行，0 = 固件没这条命令"""
        self.mb.write_holding([CMD_TYPE_SYS, SYS_SET_POWER,
                               1 if vdd_on else 0, 1 if vpp_on else 0, 1 if os_on else 0],
                              timeout=2.0)
        return self.mb.read_input(1)[0]

    def power_on(self, mv=None):
        if mv is not None:
            self.set_vdd(mv)
        return self.set_power(True)

    def power_off(self):
        return self.set_power(False)

    def run_ft(self, cmp_mode=0):
        """跑一次固件的 2200 终测（0x206）：它会自己上电、I2C 读系数、SPI 读气压温度、再断电"""
        self.mb.write_holding([CMD_TYPE_SYS, SYS_2200_FT, cmp_mode], timeout=10.0)
        return self.mb.read_input(49)


# ----------------------------------------------------------------------------
# 界面
# ----------------------------------------------------------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP_TITLE)

        self.mb = ModbusRtuMaster()
        self.board = Board(self.mb)
        # 系统缩放：显式像素（列宽、行高等）按它换算，否则高 DPI 下会被挤扁
        try:
            self.sc = max(1.0, self.winfo_fpixels("1i") / 96.0)
        except Exception:
            self.sc = 1.0

        self.port_map = {}                 # 显示名 -> COM 口
        self.log_lock = threading.Lock()
        self.auto_refresh = False
        self.stab_running = False
        self.stab_stop = threading.Event()
        self.stab_q = queue.Queue()
        self.inited = False
        self.powered = None          # None=未知 / True=已上电 / False=已断电（SPI 必读 00）

        self._build_style()
        self._build_connection()
        self._build_device()
        self._build_tabs()
        self._build_statusbar()

        self.refresh_ports()
        self._fit_window()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _fit_window(self):
        """按控件实际需要定窗口大小，避免不同 DPI 缩放下右侧被截断"""
        self.update_idletasks()
        w = self.winfo_reqwidth() + 16
        h = self.winfo_reqheight() + 16
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        w = min(w, int(sw * 0.95))
        h = min(h, int(sh * 0.92))
        self.geometry("%dx%d" % (w, h))
        self.minsize(760, 560)

    # ---------------- 外观 ----------------
    def _build_style(self):
        st = ttk.Style(self)
        try:
            st.theme_use("vista")
        except Exception:
            pass
        st.configure("TLabelframe.Label", font=FONT_UI)
        st.configure("TButton", font=FONT_UI)
        st.configure("TLabel", font=FONT_UI)
        st.configure("TCheckbutton", font=FONT_UI)
        st.configure("Status.TLabel", font=FONT_UI, foreground="#444444")
        # 表格里的中文必须用带中文字形的字体，并且行高按字体实际行距给，否则会上下重叠
        import tkinter.font as tkfont
        self.f_tree = tkfont.Font(family="Microsoft YaHei UI", size=9)
        st.configure("Treeview", font=self.f_tree,
                     rowheight=self.f_tree.metrics("linespace") + int(8 * self.sc))
        st.configure("Treeview.Heading", font=self.f_tree)

    # ---------------- 串口连接 ----------------
    def _build_connection(self):
        f = ttk.LabelFrame(self, text=" 串口连接 ")
        f.pack(fill="x", padx=10, pady=(10, 4))
        f.columnconfigure(1, weight=1)

        ttk.Label(f, text="端口").grid(row=0, column=0, padx=(8, 4), pady=(8, 4), sticky="e")
        self.cbo_port = ttk.Combobox(f, width=24, state="readonly", font=FONT_UI)
        self.cbo_port.grid(row=0, column=1, padx=2, pady=(8, 4), sticky="ew")
        ttk.Button(f, text="刷新", width=6, command=self.refresh_ports).grid(row=0, column=2, padx=4, pady=(8, 4))

        ttk.Label(f, text="波特率").grid(row=0, column=3, padx=(14, 4), pady=(8, 4), sticky="e")
        self.cbo_baud = ttk.Combobox(f, width=7, state="readonly", font=FONT_UI,
                                     values=["9600", "19200", "38400", "57600", "115200"])
        self.cbo_baud.current(0)
        self.cbo_baud.grid(row=0, column=4, padx=2, pady=(8, 4))

        ttk.Label(f, text="从站地址").grid(row=0, column=5, padx=(14, 4), pady=(8, 4), sticky="e")
        self.spn_slave = ttk.Spinbox(f, from_=1, to=247, width=5, font=FONT_UI)
        self.spn_slave.set(1)
        self.spn_slave.grid(row=0, column=6, padx=2, pady=(8, 4))

        self.btn_conn = ttk.Button(f, text="连接", width=8, command=self.toggle_connection)
        self.btn_conn.grid(row=0, column=7, padx=(14, 10), pady=(8, 4))

        self.lbl_conn = ttk.Label(f, text="未连接", foreground="#A32D2D", font=FONT_UI)
        self.lbl_conn.grid(row=1, column=0, columnspan=8, sticky="w", padx=10, pady=(0, 6))

    # ---------------- 设备信息 ----------------
    def _build_device(self):
        f = ttk.LabelFrame(self, text=" 设备 ")
        f.pack(fill="x", padx=10, pady=4)

        ttk.Label(f, text="固件版本").grid(row=0, column=0, padx=(8, 4), pady=8, sticky="e")
        self.lbl_ver = ttk.Label(f, text="—", font=("Consolas", 11, "bold"),
                                 foreground="#185FA5", width=7)
        self.lbl_ver.grid(row=0, column=1, padx=(2, 4), sticky="w")
        ttk.Button(f, text="读取", width=6, command=self.do_get_version).grid(row=0, column=2, padx=4)

        ttk.Separator(f, orient="vertical").grid(row=0, column=3, sticky="ns", padx=10)

        ttk.Label(f, text="VDD 设定").grid(row=0, column=4, padx=(2, 4), sticky="e")
        self.spn_vdd = ttk.Spinbox(f, from_=0, to=6000, increment=100, width=7, font=FONT_UI)
        self.spn_vdd.set(3300)
        self.spn_vdd.grid(row=0, column=5, padx=2)
        ttk.Label(f, text="mV").grid(row=0, column=6, padx=(0, 6), sticky="w")
        ttk.Button(f, text="设定", width=5, command=self.do_set_vdd).grid(row=0, column=7, padx=2)

        qf = ttk.Frame(f)
        qf.grid(row=0, column=8, padx=(6, 4))
        for mv in (3300, 2500, 1800):
            ttk.Button(qf, text=str(mv), width=5,
                       command=lambda v=mv: self.do_set_vdd(v)).pack(side="left", padx=2)

        p = ttk.LabelFrame(self, text=" 电源（传感器 VDD 由 VDD_PU 开关控制，DAC 只决定电压） ")
        p.pack(fill="x", padx=10, pady=(0, 4))
        ttk.Button(p, text="上电", width=6, command=self.do_power_on
                   ).grid(row=0, column=0, padx=(8, 4), pady=8)
        ttk.Button(p, text="断电", width=6, command=self.do_power_off
                   ).grid(row=0, column=1, padx=4, pady=8)
        ttk.Button(p, text="断电复位", width=9, command=self.do_power_cycle
                   ).grid(row=0, column=2, padx=4, pady=8)
        self.lbl_pwr = ttk.Label(p, text="未上电 —— 传感器没电时 SPI 一定读 00",
                                 foreground="#A32D2D", font=FONT_UI)
        self.lbl_pwr.grid(row=0, column=3, padx=(12, 10), sticky="w")

    # ---------------- 主体 ----------------
    def _build_tabs(self):
        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True, padx=10, pady=4)
        self._tab_spi()
        self._tab_regs()
        self._tab_stability()
        self._tab_log()

    def _tab_spi(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f, text="  SPI 收发  ")

        flow = ttk.LabelFrame(f, text=" 推荐流程 ")
        flow.pack(fill="x", padx=8, pady=(8, 4))
        ttk.Button(flow, text="① 一键初始化（上电 + 切 4 线 + 校验）", width=32,
                   command=self.do_init_sequence).grid(row=0, column=0, padx=(10, 6), pady=8, sticky="w")
        ttk.Button(flow, text="② 0x206 快速验证", width=16,
                   command=self.do_ft_check).grid(row=0, column=1, padx=6, pady=8, sticky="w")
        ttk.Label(flow, text="（② 不用改固件就能判断 SPI 通不通）", foreground="#666666",
                  font=("Microsoft YaHei UI", 8)).grid(row=0, column=2, padx=(4, 10), sticky="w")

        box = ttk.LabelFrame(f, text=" 手动事务（全双工：发几个字节就同时收几个字节） ")
        box.pack(fill="x", padx=8, pady=8)
        box.columnconfigure(1, weight=1)

        ttk.Label(box, text="发送字节 (HEX)").grid(row=0, column=0, padx=(10, 6), pady=8, sticky="e")
        self.txt_tx = ttk.Entry(box, width=40, font=FONT_MONO)
        self.txt_tx.grid(row=0, column=1, padx=2, pady=8, sticky="ew")
        self.txt_tx.insert(0, "80 06 FF")
        ttk.Button(box, text="发送", width=8, command=self.do_spi_send).grid(row=0, column=2, padx=8)

        ttk.Label(box, text="最多 %d 字节；读寄存器发 80 <reg> FF，rx[2] 才是数据"
                  % SPI_MAX_BYTES, foreground="#666666", font=("Microsoft YaHei UI", 8)
                  ).grid(row=1, column=1, columnspan=2, sticky="w", pady=(0, 8))

        quick = ttk.LabelFrame(f, text=" 快捷操作 ")
        quick.pack(fill="x", padx=8, pady=4)

        ttk.Button(quick, text="① 切 4 线 SPI", width=14, command=self.do_4wire
                   ).grid(row=0, column=0, padx=(10, 6), pady=8, sticky="w")

        ttk.Label(quick, text="寄存器").grid(row=1, column=0, padx=(10, 6), sticky="e")
        self.txt_reg = ttk.Entry(quick, width=8, font=FONT_MONO)
        self.txt_reg.insert(0, "06")
        self.txt_reg.grid(row=1, column=1, sticky="w")
        ttk.Button(quick, text="读", width=6, command=self.do_read_reg).grid(row=1, column=2, padx=6)
        self.lbl_reg_val = ttk.Label(quick, text="—", font=("Consolas", 12, "bold"), foreground="#185FA5")
        self.lbl_reg_val.grid(row=1, column=3, padx=(6, 20), sticky="w")

        ttk.Label(quick, text="写寄存器").grid(row=2, column=0, padx=(10, 6), pady=(0, 8), sticky="e")
        self.txt_wreg = ttk.Entry(quick, width=8, font=FONT_MONO)
        self.txt_wreg.insert(0, "00")
        self.txt_wreg.grid(row=2, column=1, pady=(0, 8), sticky="w")
        self.txt_wval = ttk.Entry(quick, width=8, font=FONT_MONO)
        self.txt_wval.insert(0, "01")
        self.txt_wval.grid(row=2, column=2, pady=(0, 8), sticky="w")
        ttk.Button(quick, text="写", width=6, command=self.do_write_reg).grid(row=2, column=3, padx=6, pady=(0, 8), sticky="w")

        ttk.Button(quick, text="自环校验 (PC6↔PA8)", width=20, command=self.do_loopback
                   ).grid(row=0, column=1, columnspan=3, padx=6, pady=8, sticky="w")

        res = ttk.LabelFrame(f, text=" 结果 ")
        res.pack(fill="both", expand=True, padx=8, pady=(4, 8))
        self.txt_res = ScrolledText(res, height=14, font=FONT_MONO, wrap="none",
                                    bg="#FBFBFB", relief="flat")
        self.txt_res.pack(fill="both", expand=True, padx=6, pady=6)
        self.txt_res.configure(state="disabled")

    def _tab_regs(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f, text="  寄存器监视  ")

        bar = ttk.Frame(f)
        bar.pack(fill="x", padx=8, pady=8)
        ttk.Button(bar, text="读取全部", width=10, command=self.do_read_all).pack(side="left")
        ttk.Button(bar, text="触发转换并读", width=13, command=self.do_measure
                   ).pack(side="left", padx=8)
        self.var_auto = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="自动刷新", variable=self.var_auto,
                        command=self.toggle_auto).pack(side="left", padx=12)
        ttk.Label(bar, text="间隔(ms)").pack(side="left", padx=(8, 4))
        self.spn_period = ttk.Spinbox(bar, from_=100, to=5000, increment=100, width=7)
        self.spn_period.set(500)
        self.spn_period.pack(side="left")
        self.lbl_sum = ttk.Label(bar, text="气压 —   温度 —", font=("Consolas", 11, "bold"),
                                 foreground="#185FA5")
        self.lbl_sum.pack(side="left", padx=20)

        cols = ("reg", "hex", "dec", "note")
        self.tree = ttk.Treeview(f, columns=cols, show="headings", height=14)
        for c, t, w, a in (("reg", "寄存器", 100, "center"), ("hex", "值 (HEX)", 120, "center"),
                           ("dec", "值 (DEC)", 120, "center"), ("note", "说明", 420, "w")):
            self.tree.heading(c, text=t)
            self.tree.column(c, width=int(w * self.sc), anchor=a,
                             stretch=(c == "note"))
        self.tree.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        rows = [(REG_STATUS, "状态寄存器，bit0 = 气压转换完成"),
                (REG_P_MSB, "气压 24 位 高字节"),
                (REG_P_MID, "气压 24 位 中字节"),
                (REG_P_LSB, "气压 24 位 低字节"),
                (REG_T_MSB, "温度 16 位 高字节"),
                (REG_T_LSB, "温度 16 位 低字节")]
        for reg, note in rows:
            self.tree.insert("", "end", iid="%02X" % reg,
                             values=("0x%02X" % reg, "—", "—", note))

    def _tab_stability(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f, text="  稳定性测试  ")

        bar = ttk.Frame(f)
        bar.pack(fill="x", padx=8, pady=8)
        ttk.Label(bar, text="次数").pack(side="left", padx=(0, 4))
        self.spn_cnt = ttk.Spinbox(bar, from_=1, to=100000, width=8)
        self.spn_cnt.set(100)
        self.spn_cnt.pack(side="left")
        ttk.Label(bar, text="间隔(ms)").pack(side="left", padx=(12, 4))
        self.spn_gap = ttk.Spinbox(bar, from_=0, to=5000, width=7)
        self.spn_gap.set(50)
        self.spn_gap.pack(side="left")
        self.btn_stab = ttk.Button(bar, text="开始", width=8, command=self.toggle_stability)
        self.btn_stab.pack(side="left", padx=16)

        ttk.Label(bar, text="每轮读 0x06/0x07/0x08 组成气压 24 位，统计失败率与波动",
                  foreground="#666666", font=("Microsoft YaHei UI", 8)).pack(side="left", padx=8)

        self.txt_stab = ScrolledText(f, height=18, font=FONT_MONO, wrap="none",
                                     bg="#FBFBFB", relief="flat")
        self.txt_stab.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.txt_stab.configure(state="disabled")

    def _tab_log(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f, text="  日志  ")

        bar = ttk.Frame(f)
        bar.pack(fill="x", padx=8, pady=8)
        ttk.Button(bar, text="清空", width=8, command=self.clear_log).pack(side="left")
        ttk.Button(bar, text="保存", width=8, command=self.save_log).pack(side="left", padx=8)

        self.txt_log = ScrolledText(f, height=20, font=FONT_MONO, wrap="none",
                                    bg="#FBFBFB", relief="flat")
        self.txt_log.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.txt_log.configure(state="disabled")

    def _build_statusbar(self):
        self.lbl_status = ttk.Label(self, text="就绪", style="Status.TLabel", anchor="w")
        self.lbl_status.pack(fill="x", padx=12, pady=(0, 8))

    # ---------------- 通用 ----------------
    def set_status(self, text):
        self.lbl_status.configure(text=text)

    def log(self, text, direction=""):
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = "%s  %s%s\n" % (ts, ("%s " % direction) if direction else "", text)
        with self.log_lock:
            self.txt_log.configure(state="normal")
            self.txt_log.insert("end", line)
            self.txt_log.see("end")
            self.txt_log.configure(state="disabled")

    def append_result(self, text):
        self.txt_res.configure(state="normal")
        self.txt_res.insert("end", text + "\n")
        self.txt_res.see("end")
        self.txt_res.configure(state="disabled")

    def append_stab(self, text):
        self.txt_stab.configure(state="normal")
        self.txt_stab.insert("end", text + "\n")
        self.txt_stab.see("end")
        self.txt_stab.configure(state="disabled")

    # ---------------- 连接 ----------------
    def refresh_ports(self):
        self.port_map.clear()
        items = []
        if serial is not None:
            for p in serial.tools.list_ports.comports():
                desc = "%s  %s" % (p.device, p.description or "")
                self.port_map[desc.strip()] = p.device
                items.append(desc.strip())
        self.cbo_port["values"] = items
        if items and not self.cbo_port.get():
            self.cbo_port.current(0)
        if not items:
            self.set_status("没有找到串口，请插好 USB 转串口线后点刷新")

    def toggle_connection(self):
        if self.mb.is_open:
            self.mb.close()
            self.btn_conn.configure(text="连接")
            self.lbl_conn.configure(text="未连接", foreground="#A32D2D")
            self.set_status("串口已断开")
            self.log("串口已断开")
            return
        desc = self.cbo_port.get()
        if not desc:
            messagebox.showwarning("提示", "请先选择串口")
            return
        port = self.port_map.get(desc, desc.split()[0])
        try:
            self.mb.open(port, int(self.cbo_baud.get()), int(self.spn_slave.get()))
        except Exception as e:
            messagebox.showerror("打开串口失败", str(e))
            return
        self.btn_conn.configure(text="断开")
        self.lbl_conn.configure(text="已连接 %s" % port, foreground="#3B6D11")
        self.set_status("已连接 %s @ %s 8N1，从站 %s" % (port, self.cbo_baud.get(), self.spn_slave.get()))
        self.log("连接 %s @ %s 8N1，从站地址 %s" % (port, self.cbo_baud.get(), self.spn_slave.get()))

    def require_conn(self) -> bool:
        if not self.mb.is_open:
            messagebox.showwarning("提示", "请先连接串口")
            return False
        return True

    def require_power(self, action="SPI") -> bool:
        """已确认断电时直接拦住：断电状态下 SPI 读回必然全是 00，没有任何诊断价值"""
        if self.powered is False:
            self.append_result("已阻止：传感器当前处于断电状态（SPI 读回必然是 00）\n"
                               "  请先点 [① 一键初始化]，或点 [上电]\n")
            self.set_status("传感器未上电，%s 已取消" % action)
            return False
        return True

    # ---------------- 设备 ----------------
    def do_get_version(self):
        if not self.require_conn():
            return
        try:
            major, minor = self.board.get_version()
        except Exception as e:
            self.handle_err("读取固件版本", e)
            return
        text = "v%d.%d" % (major, minor)
        self.lbl_ver.configure(text=text)
        self.log("读取固件版本 -> %s" % text, "RX")
        self.set_status("固件版本 %s" % text)

    def do_set_vdd(self, mv=None):
        if not self.require_conn():
            return
        if mv is None:
            try:
                mv = int(float(self.spn_vdd.get()))
            except ValueError:
                messagebox.showwarning("提示", "VDD 要填数字（单位 mV）")
                return
        if mv > 6000 and not messagebox.askyesno(
                "确认", "VDD 填了 %d mV，固件按 mV+200 补偿二极管压降后送 DAC，"
                        "超过约 6450 mV 会削顶。确定要发吗？" % mv):
            return
        try:
            self.board.set_vdd(mv)
        except Exception as e:
            self.handle_err("设置 VDD", e)
            return
        self.spn_vdd.set(mv)
        self.log("设置 VDD = %d mV（只改 DAC 电压，没上电的话传感器仍然无电）" % mv, "TX")
        self.set_status("VDD 设为 %d mV —— 还要点[上电]才会真正接通，或直接用[一键初始化]" % mv)

    def do_power_cycle(self):
        if not self.require_conn():
            return
        target = self.board.last_vdd or 3300
        ack = self._power(False)
        if ack != 1:
            # 固件没有 0x208 时退化成“把 DAC 拉到 0 再拉回”，只能模拟
            try:
                self.board.set_vdd(0)
                time.sleep(0.30)
                self.board.set_vdd(target)
            except Exception as e:
                self.handle_err("断电复位", e)
                return
            self.log("断电复位（DAC 拉 0 模拟）：0 -> %d mV" % target, "TX")
            self.set_status("已用 DAC 拉 0 模拟断电（固件无电源命令，VDD_PU 并未真正关断）")
            return
        time.sleep(0.30)
        self._power(True)
        self.log("断电复位：VDD_PU 0 -> 1（%d mV）" % target, "TX")
        self.set_status("已断电复位，芯片回到上电默认状态")

    def _power(self, on: bool):
        """真正控 VDD_PU。返回 1 = 固件支持并执行，0 = 固件没这条命令"""
        if not self.require_conn():
            return None
        try:
            ack = self.board.set_power(on)
        except Exception as e:
            self.handle_err("上电" if on else "断电", e)
            return None
        if ack == 1:
            self.powered = bool(on)
            self.lbl_pwr.configure(
                text=("已上电 %d mV" % (self.board.last_vdd or 0)) if on else "已断电",
                foreground="#3B6D11" if on else "#A32D2D")
        else:
            self.lbl_pwr.configure(text="固件没有 0x208 电源命令", foreground="#A32D2D")
            self.append_result("电源命令未被固件支持（回读 %s）。按 README 的“固件补丁”加 3 行，\n"
                               "或先用 [0x206 快速验证] 判断 SPI 通不通。\n" % ack)
        return ack

    def do_power_on(self):
        if not self.require_conn():
            return
        try:
            mv = int(float(self.spn_vdd.get()))
        except ValueError:
            messagebox.showwarning("提示", "VDD 要填数字（mV）")
            return
        try:
            self.board.set_vdd(mv)
        except Exception as e:
            self.handle_err("设置 VDD", e)
            return
        if self._power(True) == 1:
            self.log("上电：VDD=%d mV，VDD_PU=1" % mv, "TX")
            self.set_status("传感器已上电 %d mV（上电后等 100 ms 再发 SPI）" % mv)

    def do_power_off(self):
        if self.require_conn() and self._power(False) == 1:
            self.log("断电：VDD_PU=0", "TX")
            self.set_status("传感器已断电，芯片回到上电默认状态（I2C 模式）")

    # ---------------- SPI ----------------
    def do_spi_send(self):
        if not self.require_conn() or not self.require_power("手动事务"):
            return
        try:
            tx = parse_hex(self.txt_tx.get())
        except Exception as e:
            messagebox.showwarning("字节格式不对", str(e))
            return
        if not tx:
            return
        self._do_spi(tx, "手动事务")

    def _do_spi(self, tx: bytes, tag: str) -> bytes:
        t0 = time.time()
        try:
            rx = self.board.spi(tx)
        except Exception as e:
            self.handle_err(tag, e)
            raise
        dt = (time.time() - t0) * 1000
        self.append_result("[%s] %s\n   TX : %s\n   RX : %s\n   耗时 %.0f ms\n"
                           % (datetime.datetime.now().strftime("%H:%M:%S"), tag,
                              to_hex(tx), to_hex(rx), dt))
        self.log("%s  TX %s | RX %s" % (tag, to_hex(tx), to_hex(rx)), "SPI")
        self.set_status("%s 完成，RX = %s" % (tag, to_hex(rx)))
        return rx

    def do_4wire(self):
        if not self.require_conn() or not self.require_power("切 4 线"):
            return
        try:
            self._do_spi(bytes((0x00, 0x00, 0x01)), "切 4 线 SPI")
        except Exception:
            return
        self.set_status("已发 4 线切换命令，芯片会一直停在 SPI 模式，直到断电（可用“断电复位”）")

    def do_read_reg(self):
        if not self.require_conn() or not self.require_power("读寄存器"):
            return
        try:
            reg = int(self.txt_reg.get().strip(), 16)
        except ValueError:
            messagebox.showwarning("提示", "寄存器填十六进制，例如 06")
            return
        try:
            rx = self._do_spi(bytes((0x80, reg & 0xFF, 0xFF)), "读寄存器 0x%02X" % reg)
        except Exception:
            return
        self.lbl_reg_val.configure(text="0x%02X" % rx[2])

    def do_write_reg(self):
        if not self.require_conn() or not self.require_power("写寄存器"):
            return
        try:
            reg = int(self.txt_wreg.get().strip(), 16)
            val = int(self.txt_wval.get().strip(), 16)
        except ValueError:
            messagebox.showwarning("提示", "寄存器和值都填十六进制")
            return
        try:
            self._do_spi(bytes((0x00, reg & 0xFF, val & 0xFF)), "写寄存器 0x%02X" % reg)
        except Exception:
            return

    def do_loopback(self):
        if not self.require_conn():
            return
        if not messagebox.askokcancel(
                "自环校验",
                "使用前请先：\n"
                "  1) 让传感器停止供电（点 0 或直接拔掉传感器）\n"
                "  2) 用飞线把 PC6(MOSI) 和 PA8(MISO) 短接\n"
                "否则传感器会在 CS 拉低时驱动 MISO，两边顶线。\n\n"
                "程序会发 00 / FF / 55 / AA / 0F 并按位比对回读值。\n继续？"):
            return
        patterns = [bytes((0x00,)), bytes((0xFF,)), bytes((0x55,)),
                    bytes((0xAA,)), bytes((0x0F,)), bytes((0x00, 0xFF, 0x55, 0xAA, 0x0F))]
        ok = 0
        self.append_result("=== 自环校验（PC6 -> PA8）===")
        for tx in patterns:
            try:
                rx = self.board.spi(tx)
            except Exception as e:
                self.append_result("   TX %-14s 失败：%s" % (to_hex(tx), e))
                continue
            good = (rx == tx)
            ok += 1 if good else 0
            self.append_result("   %s  TX %-14s RX %-14s" % ("PASS" if good else "FAIL", to_hex(tx), to_hex(rx)))
        self.append_result("   结果：%d / %d 通过\n" % (ok, len(patterns)))
        self.log("自环校验 %d/%d 通过" % (ok, len(patterns)), "SPI")
        self.set_status("自环校验 %d/%d 通过" % (ok, len(patterns)))

    def do_init_sequence(self):
        """上电 -> 等稳定 -> 写 0x00=0x01 切 4 线（回读校验，最多 5 次） -> 读 Part ID"""
        if not self.require_conn():
            return
        try:
            mv = int(float(self.spn_vdd.get()))
        except ValueError:
            mv = 3300

        self.append_result("=== 一键初始化 ===")
        # 1) 设压 + 上电（真正让 VDD_PU 接通，光设 DAC 是不供电的）
        try:
            self.board.set_vdd(mv)
            ack = self.board.set_power(True)
        except Exception as e:
            self.append_result("  上电失败：%s\n" % e)
            self.handle_err("一键初始化", e)
            return
        if ack != 1:
            self.append_result("  固件不支持 0x208 电源命令（回读 %s），传感器仍是断电状态" % ack)
            self.append_result("  → 先点 [0x206 快速验证] 判断 SPI 通不通，")
            self.append_result("    或按 README 的“固件补丁”给固件加 3 行电源命令\n")
            self.lbl_pwr.configure(text="固件没有 0x208 电源命令", foreground="#A32D2D")
            return
        self.lbl_pwr.configure(text="已上电 %d mV" % mv, foreground="#3B6D11")
        self.powered = True
        self.append_result("  上电 OK：VDD = %d mV，等 150 ms 稳定" % mv)
        time.sleep(0.15)

        # 2) 切 4 线：写 0x00=0x01 后必须回读确认（老 F103 固件也是这么做的）
        ok = False
        reg0 = None
        for i in range(5):
            try:
                self.board.spi(bytes((0x00, 0x00, 0x01)))
                time.sleep(0.005)
                reg0 = self.board.spi_read_reg(0x00)
            except Exception as e:
                self.append_result("  第 %d 次切换出错：%s\n" % (i + 1, e))
                break
            self.append_result("  第 %d 次切 4 线，回读 0x00 = 0x%02X" % (i + 1, reg0))
            if reg0 == 0x01:
                ok = True
                break
            time.sleep(0.02)

        if not ok:
            self.append_result("  [FAIL] 0x00 回读不是 0x01，4 线模式没切成功")
            if reg0 == 0x00:
                self.append_result("    回读 0x00 → MISO 被钳在低电平：芯片没电 / MISO 短地 / SDO 没接")
            elif reg0 == 0xFF:
                self.append_result("    回读 0xFF → MISO 悬空：SDO 没接到 PA8，或座上没芯片")
            self.append_result("    先用 [0x206 快速验证] 交叉确认，再量排线 VDD 和 MISO\n")
            self.set_status("初始化失败：4 线 SPI 没切成功")
            self.log("一键初始化失败，0x00 回读 0x%02X" % (reg0 if reg0 is not None else 0), "SPI")
            return

        # 3) Part ID，确认芯片真的在应答
        try:
            pid = self.board.spi_read_reg(0x01)
        except Exception as e:
            self.append_result("  读 Part ID 失败：%s\n" % e)
            return
        self.append_result("  [OK] 4 线 SPI 已生效，Part ID(0x01) = 0x%02X" % pid)
        self.append_result("  → 现在可以读 0x02 / 0x06~0x08 / 0x09~0x0A 了\n")
        self.inited = True
        self.log("一键初始化成功，Part ID = 0x%02X" % pid, "SPI")
        self.set_status("初始化成功：4 线 SPI 已生效，Part ID 0x%02X" % pid)

    def do_ft_check(self):
        """跑固件自带的 0x206 终测：它会自己上电、I2C 读系数、SPI 读气压温度、再断电。
        注意它的前半段是 I2C，必须让芯片处于上电默认的 I2C 态，所以先断电复位。"""
        if not self.require_conn():
            return
        self.append_result("=== 0x206 快速验证（固件自身上电 → I2C → SPI → 断电）===")

        # 前提：芯片必须在 I2C 态。刚做过一键初始化的芯片停在 4 线 SPI，
        # 直接跑 0x206 会在第一段 I2C 读写就 NACK（err=0x01），SPI 根本轮不到。
        try:
            mv = int(float(self.spn_vdd.get()))
        except ValueError:
            mv = 3300
        try:
            self.board.set_vdd(mv)
            ack = self.board.set_power(False)
            if ack == 1:
                time.sleep(0.30)
                self.board.set_power(True)
                time.sleep(0.15)
                self.append_result("  已断电复位（VDD_PU 0→1），芯片回到 I2C 默认态")
            else:
                self.append_result("  固件没有 0x208，无法真正断电——若芯片还停在 4 线 SPI，")
                self.append_result("  下面很可能在第一段 I2C 就失败（err 非 0）")
        except Exception as e:
            self.append_result("  断电复位失败：%s" % e)
        self.inited = False
        self.lbl_pwr.configure(text="0x206 运行中（结束会自动断电）", foreground="#634806")

        try:
            regs = self.board.run_ft(0)
        except Exception as e:
            self.append_result("  执行失败：%s\n" % e)
            self.handle_err("0x206 快速验证", e)
            return
        err = regs[0]
        p = (regs[44] << 16) | (regs[45] << 8) | regs[46]
        t = (regs[47] << 8) | regs[48]
        self.append_result("  err = 0x%02X   气压 = 0x%06X (%d)   温度 = 0x%04X (%d)"
                           % (err, p, p, t, t))
        if err == 0 and p not in (0, 0xFFFFFF):
            self.append_result("  换算参考（1BAR 公式）：气压 ≈ %.2f kPa，温度 ≈ %.1f ℃"
                               % (press_kpa(p), temp_c(t)))
        self.powered = False
        if err == 0x03:
            self.append_result("  判定：SPI 读回全 FF —— SPI 没通")
        elif err == 0:
            self.append_result("  判定：SPI 通了（固件用 SPI 读到了非 FF 的气压）")
        else:
            self.append_result("  判定：I2C 段失败（err=0x%02X）。0x206 前半段是 I2C 读系数，" % err)
            self.append_result("        这块板的 I2C 可能没接/芯片不支持，但这跟 SPI 无关 ——")
            self.append_result("        验证 SPI 请直接用 [① 一键初始化] + [触发转换并读]，不必管 0x206")

        # 0x206 结尾一定会 SetIoStatus(0,0,0) 断电，这里自动恢复，免得后面又是一片 00
        self.append_result("  → 正在自动恢复：重新上电 + 切 4 线 …")
        try:
            self.board.set_vdd(mv)
            if self.board.set_power(True) == 1:
                time.sleep(0.15)
                self.board.spi(bytes((0x00, 0x00, 0x01)))
                time.sleep(0.005)
                reg0 = self.board.spi_read_reg(0x00)
                if reg0 == 0x01:
                    self.powered = True
                    self.inited = True
                    self.lbl_pwr.configure(text="已上电 %d mV（0x206 后自动恢复）" % mv,
                                           foreground="#3B6D11")
                    self.append_result("  ✓ 已恢复到 4 线 SPI（回读 0x00 = 0x01），可以直接继续测")
                else:
                    self.lbl_pwr.configure(text="已上电，但 4 线未切成功", foreground="#634806")
                    self.append_result("  ! 上电成功但 4 线回读 0x%02X，请手动点 [① 一键初始化]" % reg0)
            else:
                self.lbl_pwr.configure(text="已断电（固件无 0x208，无法自动恢复）",
                                       foreground="#A32D2D")
                self.append_result("  ! 固件没有 0x208，无法自动恢复，请手动点 [① 一键初始化]")
        except Exception as e:
            self.append_result("  ! 自动恢复失败：%s" % e)

        self.append_result("")
        self.log("0x206：err=0x%02X 气压=0x%06X 温度=0x%04X" % (err, p, t), "SPI")
        self.set_status("0x206 验证完成：err=0x%02X 气压=0x%06X" % (err, p))

    def do_measure(self):
        """按老 F103 固件的已知可行时序：触发温度 → 等 60ms → 读 0x09/0x0A；
        触发气压 → 等 60ms → 轮询 0x02.bit0 → 读 0x06/0x07/0x08"""
        if not self.require_conn() or not self.require_power("触发转换"):
            return
        if not self.inited:
            self.append_result("提示：还没做 [① 一键初始化]，读回 00/FF 都正常，先回去做那一步")
        self.append_result("=== 触发一次转换并读数 ===")
        try:
            self.board.spi(bytes((0x00, 0x30, 0x08)))      # 触发温度转换
            time.sleep(0.06)
            t_h = self.board.spi_read_reg(REG_T_MSB)
            t_l = self.board.spi_read_reg(REG_T_LSB)

            self.board.spi(bytes((0x00, 0x30, 0x09)))      # 触发气压转换
            st = 0
            for i in range(20):
                time.sleep(0.01)
                st = self.board.spi_read_reg(REG_STATUS)
                if st & 0x01:
                    break
            p_h = self.board.spi_read_reg(REG_P_MSB)
            p_m = self.board.spi_read_reg(REG_P_MID)
            p_l = self.board.spi_read_reg(REG_P_LSB)
        except Exception as e:
            self.append_result("  失败：%s\n" % e)
            self.handle_err("触发转换", e)
            return

        p = (p_h << 16) | (p_m << 8) | p_l
        t = (t_h << 8) | t_l
        vals = {REG_STATUS: st, REG_P_MSB: p_h, REG_P_MID: p_m,
                REG_P_LSB: p_l, REG_T_MSB: t_h, REG_T_LSB: t_l}
        for reg, v in vals.items():
            self.tree.item("%02X" % reg,
                           values=("0x%02X" % reg, "0x%02X" % v, str(v), self._note(reg)))
        self.lbl_sum.configure(text=fmt_reading(p, t))

        self.append_result("  状态 0x02 = 0x%02X（bit0=%d 表示气压转换完成）" % (st, st & 1))
        self.append_result("  气压 = 0x%06X (%d) ≈ %.2f kPa   温度 = 0x%04X (%d) ≈ %.1f ℃"
                           % (p, p, press_kpa(p), t, t, temp_c(t)))
        if p == 0xFFFFFF:
            self.append_result("  [FAIL] 气压全 FF —— SDO 没输出/没切 4 线")
        elif p == 0:
            self.append_result("  [FAIL] 气压全 00 —— MISO 被钳低（没电/短地）")
        else:
            self.append_result("  [OK] 拿到非平凡数值，SPI 数据通路正常")
        self.append_result("")
        self.log("触发转换：状态 0x%02X 气压 0x%06X 温度 0x%04X" % (st, p, t), "SPI")
        self.set_status("测量完成：气压 0x%06X 温度 0x%04X" % (p, t))

    # ---------------- 寄存器监视 ----------------
    def do_read_all(self):
        if not self.require_conn() or not self.require_power("读取全部"):
            return
        if not self.inited:
            self.append_result("提示：还没做 [① 一键初始化]，读回全 00 / 全 FF 都属正常")
        try:
            vals = {}
            for reg in (REG_STATUS, REG_P_MSB, REG_P_MID, REG_P_LSB, REG_T_MSB, REG_T_LSB):
                vals[reg] = self.board.spi_read_reg(reg)
        except Exception as e:
            self.handle_err("读取寄存器", e)
            return
        for reg, v in vals.items():
            self.tree.item("%02X" % reg, values=("0x%02X" % reg, "0x%02X" % v, str(v), self._note(reg)))
        p = (vals[REG_P_MSB] << 16) | (vals[REG_P_MID] << 8) | vals[REG_P_LSB]
        t = (vals[REG_T_MSB] << 8) | vals[REG_T_LSB]
        self.lbl_sum.configure(text=fmt_reading(p, t))
        self.log("读全部寄存器：状态 %02X 气压 %06X 温度 %04X"
                 % (vals[REG_STATUS], p, t), "SPI")
        self.set_status("寄存器读取完成：气压 ≈ %.2f kPa，温度 ≈ %.1f ℃" % (press_kpa(p), temp_c(t)))

    @staticmethod
    def _note(reg):
        return {REG_STATUS: "状态寄存器，bit0 = 气压转换完成",
                REG_P_MSB: "气压 24 位 高字节", REG_P_MID: "气压 24 位 中字节",
                REG_P_LSB: "气压 24 位 低字节", REG_T_MSB: "温度 16 位 高字节",
                REG_T_LSB: "温度 16 位 低字节"}.get(reg, "")

    def toggle_auto(self):
        self.auto_refresh = self.var_auto.get()
        if self.auto_refresh:
            self._auto_tick()

    def _auto_tick(self):
        if not self.auto_refresh:
            return
        if self.mb.is_open:
            try:
                vals = {r: self.board.spi_read_reg(r) for r in
                        (REG_STATUS, REG_P_MSB, REG_P_MID, REG_P_LSB, REG_T_MSB, REG_T_LSB)}
                for reg, v in vals.items():
                    self.tree.item("%02X" % reg, values=("0x%02X" % reg, "0x%02X" % v, str(v), self._note(reg)))
                p = (vals[REG_P_MSB] << 16) | (vals[REG_P_MID] << 8) | vals[REG_P_LSB]
                t = (vals[REG_T_MSB] << 8) | vals[REG_T_LSB]
                self.lbl_sum.configure(text=fmt_reading(p, t))
            except Exception as e:
                self.var_auto.set(False)
                self.auto_refresh = False
                self.handle_err("自动刷新", e)
                return
        try:
            period = max(100, int(self.spn_period.get()))
        except ValueError:
            period = 500
        self.after(period, self._auto_tick)

    # ---------------- 稳定性 ----------------
    def toggle_stability(self):
        if self.stab_running:
            # 只是请求停止：让线程把最后一份统计发回来，由 _stab_finish 复位按钮
            self.stab_stop.set()
            self.btn_stab.configure(text="停止中")
            return
        if not self.require_conn():
            return
        try:
            cnt = int(self.spn_cnt.get())
            gap = int(self.spn_gap.get())
        except ValueError:
            messagebox.showwarning("提示", "次数/间隔要填数字")
            return
        self.stab_running = True
        self.stab_stop.clear()
        self.btn_stab.configure(text="停止")
        self.txt_stab.configure(state="normal")
        self.txt_stab.delete("1.0", "end")
        self.txt_stab.configure(state="disabled")
        self.stab_q = queue.Queue()
        threading.Thread(target=self._stab_worker, args=(cnt, gap), daemon=True).start()
        self.after(100, self._poll_stab)

    def _poll_stab(self):
        try:
            while True:
                kind, payload = self.stab_q.get_nowait()
                if kind == "line":
                    self.append_stab(payload)
                elif kind == "done":
                    self._stab_finish(*payload)
                    return
        except queue.Empty:
            pass
        if self.stab_running:
            self.after(100, self._poll_stab)

    def _stab_worker(self, cnt, gap):
        ok = fail = 0
        vals = []
        errs = {}
        t_start = time.time()
        for i in range(cnt):
            if self.stab_stop.is_set():
                break
            try:
                b0 = self.board.spi_read_reg(REG_P_MSB)
                b1 = self.board.spi_read_reg(REG_P_MID)
                b2 = self.board.spi_read_reg(REG_P_LSB)
                v = (b0 << 16) | (b1 << 8) | b2
                if v == 0xFFFFFF or v == 0x000000:
                    fail += 1
                    errs["气压全 %02X" % b0] = errs.get("气压全 %02X" % b0, 0) + 1
                else:
                    ok += 1
                    vals.append(v)
                if i == 0 or (i + 1) % 10 == 0:
                    self.stab_q.put(("line", "  #%d  %s  气压 0x%06X (%d)"
                                     % (i + 1, "FAIL" if v in (0xFFFFFF, 0x000000) else "OK", v, v)))
            except Exception as e:
                fail += 1
                key = str(e)[:30]
                errs[key] = errs.get(key, 0) + 1
                self.stab_q.put(("line", "  #%d  ERR  %s" % (i + 1, e)))
            if gap:
                time.sleep(gap / 1000.0)
        self.stab_q.put(("done", (ok, fail, vals, errs, time.time() - t_start)))

    def _stab_finish(self, ok, fail, vals, errs, spent):
        lines = ["", "=== 稳定性结果 ===",
                 "  样本：%d 次（成功 %d / 失败 %d），用时 %.1f s，单次平均 %.1f ms"
                 % (ok + fail, ok, fail, spent, spent * 1000.0 / max(1, ok + fail))]
        if vals:
            lines.append("  气压首值 0x%06X (%d)   末值 0x%06X (%d)" % (vals[0], vals[0], vals[-1], vals[-1]))
            lines.append("  最小值 %d   最大值 %d   波动 %d   (24 位满量程 0x%06X)"
                         % (min(vals), max(vals), max(vals) - min(vals), 0xFFFFFF))
            if fail == 0:
                lines.append("  判定：全部成功，SPI 链路稳定")
            else:
                lines.append("  判定：有失败样本，成功率 %.1f%%" % (100.0 * ok / max(1, ok + fail)))
        else:
            lines.append("  没有取到有效样本，SPI 完全没通")
        if errs:
            lines.append("  失败分布：" + "，".join("%s x%d" % (k, v) for k, v in errs.items()))
        self.append_stab("\n".join(lines))
        self.log("稳定性测试：成功 %d / 失败 %d / 共 %d 次" % (ok, fail, ok + fail), "SPI")
        self.set_status("稳定性测试结束：成功 %d，失败 %d" % (ok, fail))
        self.btn_stab.configure(text="开始")
        self.stab_running = False

    # ---------------- 日志 ----------------
    def clear_log(self):
        self.txt_log.configure(state="normal")
        self.txt_log.delete("1.0", "end")
        self.txt_log.configure(state="disabled")

    def save_log(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("文本", "*.txt"), ("全部", "*.*")],
            initialfile="spi_log_%s.txt" % datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as fp:
                fp.write(self.txt_log.get("1.0", "end"))
            self.set_status("日志已保存：%s" % path)
        except Exception as e:
            messagebox.showerror("保存失败", str(e))

    # ---------------- 错误 ----------------
    def handle_err(self, tag, e):
        msg = str(e)
        self.append_result("[%s] 失败：%s" % (tag, msg))
        self.log("%s 失败：%s" % (tag, msg), "ERR")
        self.set_status("%s 失败：%s" % (tag, msg))

    def on_close(self):
        self.auto_refresh = False
        self.stab_stop.set()
        try:
            self.mb.close()
        except Exception:
            pass
        self.destroy()


def main():
    if serial is None:
        root = tk.Tk()
        root.withdraw()
        messagebox.showerror("缺少依赖", "没有找到 pyserial，请先执行：pip install pyserial")
        return
    try:  # 高 DPI 下字更清晰
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    App().mainloop()


if __name__ == "__main__":
    main()
