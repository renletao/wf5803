# -*- coding: utf-8 -*-
"""SPI 采集测试工具（精简版）

固件接口（CmdHelper.c::ParseSPICmd，2026-11 改版后）：
  写 1 个寄存器到 40000：       [6]                      ← 只有命令字，无参数
  读 7 个寄存器从 30000：
      [0]    err        0x00 正常 / 0x01 全FF / 0x02 全00 / 0x03 DRDY超时
      [1]    status     芯片 0x02 寄存器（bit0 = DRDY）
      [2..4] 气压 24 位 高/中/低（0x06/0x07/0x08）
      [5..6] 温度 16 位 高/低（0x09/0x0A）

  固件内部自己完成：切 4 线 → 写 0x30=0x0A 组合触发 → 轮询 DRDY → 连读气压温度
  典型 15~25 ms，最坏约 45 ms，所以这条命令超时要给到 1 s。

  其他命令：
  读版本   写 [1, 0x0104]  读 2 个
  设压+上电 写 [1, 0x0207, mV]，读 2 个：out[0]=1 表示已执行，out[1]=实际生效的电压
            mV 传 0 表示断电（固件会断开 VDD_PU，而不是把电压设成 0V）

地址：写 40000 / 读 30000（固件回调收到 40001/30001，FreeModbus 会先 +1）
"""

import time
import datetime
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

APP_TITLE = "SPI 采集测试工具"

ADDR_W = 40000
ADDR_R = 30000

T_SYS, T_SPI = 1, 6
SYS_VERSION, SYS_VDD = 0x0104, 0x0207

ERR_TEXT = {
    0x00: ("正常", "#3B6D11"),
    0x01: ("气压全 FF：MISO 没通 / 没切 4 线 / SDO 悬空", "#A32D2D"),
    0x02: ("气压全 00：没上电 / MISO 短地", "#A32D2D"),
    0x03: ("等不到 DRDY：转换超时", "#854F0B"),
}

FONT_UI = ("Microsoft YaHei UI", 9)
FONT_MONO = ("Consolas", 10)
FONT_BIG = ("Microsoft YaHei UI", 13, "bold")


# ---------------------------------------------------------------- 协议层
def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def add_crc(body):
    c = crc16(body)
    return bytes(body) + bytes((c & 0xFF, (c >> 8) & 0xFF))


def press_kpa(raw24):
    """1BAR 换算，照抄 资料/WF5803_1BAR_IIC驱动包/wfsensor.c"""
    f = (raw24 - 16777216) / 8388608.0 if raw24 >= 8388608 else raw24 / 8388608.0
    return f * 125.0 + 17.5


def temp_c(raw16):
    return (raw16 - 65844) / 256.0 if raw16 > 32768 else (raw16 - 308) / 256.0


class ModbusError(Exception):
    pass


class Mb:
    def __init__(self):
        self.ser = None
        self.slave = 1
        self.timeout = 1.0

    @property
    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def open(self, port, baud, slave):
        self.close()
        self.ser = serial.Serial(port=port, baudrate=baud, bytesize=8,
                                 parity=serial.PARITY_NONE, stopbits=1,
                                 timeout=0.02, write_timeout=1.0)
        self.slave = int(slave)

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

    def _transact(self, body, timeout=None):
        if not self.is_open:
            raise ModbusError("串口未连接")
        to = self.timeout if timeout is None else timeout
        self.ser.reset_input_buffer()
        self.ser.write(add_crc(body))
        deadline = time.time() + to

        hdr = self._read_exact(2, deadline)
        if not hdr:
            raise ModbusError("无应答（串口/波特率/从站地址/接线）")
        if len(hdr) < 2:
            raise ModbusError("应答不完整")
        if hdr[0] != self.slave:
            raise ModbusError("从站地址不符：0x%02X" % hdr[0])

        func = hdr[1]
        if func & 0x80:
            buf = hdr + self._read_exact(3, deadline)
            if len(buf) >= 5:
                raise ModbusError("从站异常码 0x%02X" % buf[2])
            raise ModbusError("异常应答不完整")

        if func == 0x04:
            bc = self._read_exact(1, deadline)
            if not bc:
                raise ModbusError("应答不完整")
            buf = hdr + bc + self._read_exact(bc[0] + 2, deadline)
        else:
            buf = hdr + self._read_exact(6, deadline)

        c = crc16(buf[:-2])
        if bytes((c & 0xFF, c >> 8)) != buf[-2:]:
            raise ModbusError("CRC 校验失败")
        return buf

    def write_regs(self, values, timeout=None):
        n = len(values)
        body = bytearray((self.slave, 0x10, ADDR_W >> 8, ADDR_W & 0xFF,
                          (n >> 8) & 0xFF, n & 0xFF, n * 2))
        for v in values:
            v = int(v) & 0xFFFF
            body += bytes((v >> 8, v & 0xFF))
        self._transact(bytes(body), timeout)

    def read_regs(self, qty, timeout=None):
        body = bytes((self.slave, 0x04, ADDR_R >> 8, ADDR_R & 0xFF,
                      (qty >> 8) & 0xFF, qty & 0xFF))
        buf = self._transact(body, timeout)
        return [(buf[3 + 2 * i] << 8) | buf[4 + 2 * i] for i in range(qty)]


# ---------------------------------------------------------------- 界面
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP_TITLE)
        self.mb = Mb()
        self.port_map = {}

        st = ttk.Style(self)
        try:
            st.theme_use("vista")
        except Exception:
            pass
        import tkinter.font as tkfont
        for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont"):
            try:
                tkfont.nametofont(name).configure(family="Microsoft YaHei UI", size=9)
            except Exception:
                pass
        for name in ("TButton", "TLabel", "TLabelframe.Label", "TSpinbox"):
            st.configure(name, font=FONT_UI)

        self._build()
        self.refresh_ports()
        self._fit()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    @property
    def scale(self):
        try:
            return max(1.0, self.winfo_fpixels("1i") / 96.0)
        except Exception:
            return 1.0

    def _fit(self):
        self.update_idletasks()
        w = max(self.winfo_reqwidth() + 16, int(680 * self.scale))
        h = self.winfo_reqheight() + 16
        self.geometry("%dx%d" % (min(w, int(self.winfo_screenwidth() * 0.95)),
                                 min(h, int(self.winfo_screenheight() * 0.92))))
        self.minsize(int(560 * self.scale), int(400 * self.scale))

    def _build(self):
        f = ttk.LabelFrame(self, text=" 串口 ")
        f.pack(fill="x", padx=10, pady=(10, 4))
        f.columnconfigure(1, weight=1)

        ttk.Label(f, text="端口").grid(row=0, column=0, padx=(8, 4), pady=(8, 4), sticky="e")
        self.cbo_port = ttk.Combobox(f, width=22, state="readonly", font=FONT_UI)
        self.cbo_port.grid(row=0, column=1, padx=2, pady=(8, 4), sticky="ew")
        ttk.Button(f, text="刷新", width=6, command=self.refresh_ports
                   ).grid(row=0, column=2, padx=4, pady=(8, 4))

        ttk.Label(f, text="波特率").grid(row=0, column=3, padx=(12, 4), pady=(8, 4), sticky="e")
        self.cbo_baud = ttk.Combobox(f, width=7, state="readonly", font=FONT_UI,
                                     values=["9600", "19200", "115200"])
        self.cbo_baud.current(0)
        self.cbo_baud.grid(row=0, column=4, padx=2, pady=(8, 4))

        ttk.Label(f, text="从站").grid(row=0, column=5, padx=(12, 4), pady=(8, 4), sticky="e")
        self.spn_slave = ttk.Spinbox(f, from_=1, to=247, width=4, font=FONT_UI)
        self.spn_slave.set(1)
        self.spn_slave.grid(row=0, column=6, padx=2, pady=(8, 4))

        self.btn_conn = ttk.Button(f, text="连接", width=7, command=self.toggle_conn)
        self.btn_conn.grid(row=0, column=7, padx=(12, 10), pady=(8, 4))

        # ---- 设备 ----
        g = ttk.LabelFrame(self, text=" 设备 ")
        g.pack(fill="x", padx=10, pady=4)

        ttk.Label(g, text="固件版本").grid(row=0, column=0, padx=(8, 4), pady=8, sticky="e")
        self.lbl_ver = ttk.Label(g, text="—", font=("Consolas", 11, "bold"),
                                 foreground="#185FA5", width=7)
        self.lbl_ver.grid(row=0, column=1, padx=(0, 4), sticky="w")
        ttk.Button(g, text="读取", width=6, command=self.do_version
                   ).grid(row=0, column=2, padx=(0, 18))

        ttk.Label(g, text="VDD").grid(row=0, column=3, padx=(0, 4), sticky="e")
        self.spn_vdd = ttk.Spinbox(g, from_=0, to=6000, increment=100, width=6, font=FONT_UI)
        self.spn_vdd.set(3300)
        self.spn_vdd.grid(row=0, column=4)
        ttk.Label(g, text="mV").grid(row=0, column=5, padx=(2, 6), sticky="w")
        ttk.Button(g, text="上电", width=6, command=self.do_power_on
                   ).grid(row=0, column=6, padx=2)
        ttk.Button(g, text="断电", width=6, command=self.do_power_off
                   ).grid(row=0, column=7, padx=(2, 10))

        # ---- 采集 ----
        c = ttk.LabelFrame(self, text=" 采集（写 1 个寄存器 6 → 读 7 个） ")
        c.pack(fill="x", padx=10, pady=4)

        self.btn_acq = ttk.Button(c, text="采集一次", width=10, command=self.do_acquire)
        self.btn_acq.grid(row=0, column=0, rowspan=2, padx=(10, 14), pady=10, sticky="ns")

        self.lbl_press = ttk.Label(c, text="气压  —", font=FONT_BIG, foreground="#185FA5")
        self.lbl_press.grid(row=0, column=1, padx=(0, 24), pady=(10, 0), sticky="w")
        self.lbl_temp = ttk.Label(c, text="温度  —", font=FONT_BIG, foreground="#185FA5")
        self.lbl_temp.grid(row=0, column=2, padx=(0, 16), pady=(10, 0), sticky="w")

        self.lbl_raw = ttk.Label(c, text="err —   原始值 —", foreground="#5F5E5A",
                                 font=("Consolas", 9))
        self.lbl_raw.grid(row=1, column=1, columnspan=2, padx=(0, 10), pady=(2, 10), sticky="w")

        self.var_cont = tk.BooleanVar(value=False)
        cf = ttk.Frame(c)
        cf.grid(row=2, column=0, columnspan=3, sticky="w", padx=10, pady=(0, 10))
        ttk.Checkbutton(cf, text="连续采集", variable=self.var_cont,
                        command=self.toggle_cont).pack(side="left")
        ttk.Label(cf, text="间隔").pack(side="left", padx=(18, 4))
        self.spn_period = ttk.Spinbox(cf, from_=200, to=10000, increment=100, width=6,
                                      font=FONT_UI)
        self.spn_period.set(500)
        self.spn_period.pack(side="left")
        ttk.Label(cf, text="ms", foreground="#5F5E5A").pack(side="left", padx=(4, 0))

        # ---- 日志 ----
        self.txt_log = ScrolledText(self, height=10, font=FONT_MONO, wrap="none",
                                    bg="#FBFBFB", relief="flat")
        self.txt_log.pack(fill="both", expand=True, padx=10, pady=(4, 4))
        self.txt_log.configure(state="disabled")

        bar = ttk.Frame(self)
        bar.pack(fill="x", padx=10, pady=(0, 8))
        ttk.Button(bar, text="清空日志", width=9, command=self.clear_log).pack(side="left")
        self.lbl_status = ttk.Label(bar, text="未连接", foreground="#A32D2D", font=FONT_UI)
        self.lbl_status.pack(side="left", padx=12)

    # ------------------------------------------------------------ 工具
    def log(self, text):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self.txt_log.configure(state="normal")
        self.txt_log.insert("end", "%s  %s\n" % (ts, text))
        self.txt_log.see("end")
        self.txt_log.configure(state="disabled")

    def status(self, text, color="#444444"):
        self.lbl_status.configure(text=text, foreground=color)

    def clear_log(self):
        self.txt_log.configure(state="normal")
        self.txt_log.delete("1.0", "end")
        self.txt_log.configure(state="disabled")

    def refresh_ports(self):
        self.port_map.clear()
        items = []
        if serial is not None:
            for p in serial.tools.list_ports.comports():
                desc = ("%s %s" % (p.device, p.description or "")).strip()
                self.port_map[desc] = p.device
                items.append(desc)
        self.cbo_port["values"] = items
        if items and not self.cbo_port.get():
            self.cbo_port.current(0)

    def require_conn(self):
        if not self.mb.is_open:
            messagebox.showwarning("提示", "请先连接串口")
            return False
        return True

    def toggle_conn(self):
        if self.mb.is_open:
            self.mb.close()
            self.btn_conn.configure(text="连接")
            self.status("未连接", "#A32D2D")
            self.log("串口已断开")
            return
        desc = self.cbo_port.get()
        if not desc:
            messagebox.showwarning("提示", "请先选串口")
            return
        try:
            self.mb.open(self.port_map.get(desc, desc.split()[0]),
                         int(self.cbo_baud.get()), int(self.spn_slave.get()))
        except Exception as e:
            messagebox.showerror("打开串口失败", str(e))
            return
        self.btn_conn.configure(text="断开")
        self.status("已连接 %s @ %s 8N1" % (self.port_map.get(desc, desc), self.cbo_baud.get()),
                    "#3B6D11")
        self.log("连接 %s，从站 %s" % (desc, self.spn_slave.get()))

    # ------------------------------------------------------------ 指令
    def do_version(self):
        if not self.require_conn():
            return
        try:
            self.mb.write_regs([T_SYS, SYS_VERSION])
            regs = self.mb.read_regs(2)
        except Exception as e:
            self.log("[读版本] 失败：%s" % e)
            self.status("读版本失败：%s" % e, "#A32D2D")
            return
        text = "v%d.%d" % (regs[0], regs[1])
        self.lbl_ver.configure(text=text)
        self.log("[读版本] 写 1,0x0104  ->  %s" % text)
        self.status("固件版本 %s" % text, "#3B6D11")

    def _set_vdd(self, mv):
        """0x0207 = 设压 + 上电（mv=0 表示断电）。返回 (ack, 实际生效值)"""
        self.mb.write_regs([T_SYS, SYS_VDD, mv])
        regs = self.mb.read_regs(2)
        return regs[0], regs[1]

    def do_power_on(self):
        if not self.require_conn():
            return
        try:
            mv = int(float(self.spn_vdd.get()))
        except ValueError:
            messagebox.showwarning("提示", "VDD 要填数字（mV）")
            return
        if mv <= 0:
            messagebox.showwarning("提示", "上电请填大于 0 的电压（mV）")
            return
        try:
            ack, echo = self._set_vdd(mv)
        except Exception as e:
            self.log("[上电] 失败：%s" % e)
            self.status("上电失败：%s" % e, "#A32D2D")
            return
        if ack == 1:
            note = ""
            if echo != mv:
                note = "（被钳位，原填 %d）" % mv
            self.log("[上电] 0x0207 = %d mV（设压+送电）  ->  ack=1 生效 %d mV%s"
                     % (mv, echo, note))
            self.status("已上电 %d mV%s" % (echo, note), "#3B6D11")
        else:
            self.log("[上电] 0x0207 回读 %d —— 固件没执行这条命令（还是旧固件？）" % ack)
            self.status("固件未执行 0x0207", "#A32D2D")

    def do_power_off(self):
        if not self.require_conn():
            return
        try:
            ack, echo = self._set_vdd(0)
        except Exception as e:
            self.log("[断电] 失败：%s" % e)
            self.status("断电失败：%s" % e, "#A32D2D")
            return
        self.log("[断电] 0x0207 = 0（断开 VDD_PU）  ->  ack=%d" % ack)
        self.status("已断电" if ack == 1 else "固件未执行 0x0207", "#A32D2D")

    def toggle_cont(self):
        if self.var_cont.get():
            self._cont_tick()
        else:
            self.status("已停止连续采集")

    def _cont_tick(self):
        if not self.var_cont.get():
            return
        if not self.mb.is_open:
            self.var_cont.set(False)
            return
        self.do_acquire(quiet=True)
        try:
            period = max(200, int(self.spn_period.get()))
        except ValueError:
            period = 500
        self.after(period, self._cont_tick)

    def do_acquire(self, quiet=False, retries=1):
        """读 7 个寄存器并判读。err=0 但数据全 0 是"固件没更新"的特征，必须单独报出来。"""
        if not self.mb.is_open:
            if not quiet:
                messagebox.showwarning("提示", "请先连接串口")
            return

        regs = None
        err = st = p_raw = t_raw = None
        dt = 0.0
        for attempt in range(retries + 1):
            t0 = time.time()
            try:
                self.mb.write_regs([T_SPI], timeout=2.0)      # 只有命令字，无参数
                regs = self.mb.read_regs(7, timeout=2.0)
            except Exception as e:
                if not quiet:
                    self.log("[采集] 第%d次 通信失败：%s" % (attempt + 1, e))
                time.sleep(0.05)
                continue
            dt = (time.time() - t0) * 1000
            err, st = regs[0], regs[1]
            p_raw = (regs[2] << 16) | (regs[3] << 8) | regs[4]
            t_raw = (regs[5] << 8) | regs[6]
            # 只有 err=0 且数值不是平凡值才算成功，否则重试一次
            if err == 0 and p_raw not in (0, 0xFFFFFF) and t_raw not in (0, 0xFFFF):
                break
            if attempt < retries:
                time.sleep(0.05)

        if regs is None:
            self.lbl_press.configure(text="气压  —", foreground="#A32D2D")
            self.lbl_temp.configure(text="温度  —", foreground="#A32D2D")
            self.lbl_raw.configure(text="无应答")
            self.status("采集失败：通信无应答", "#A32D2D")
            return

        stub = p_raw in (0, 0xFFFFFF) or t_raw in (0, 0xFFFF)
        self.lbl_raw.configure(
            text="err 0x%02X · st 0x%02X · p 0x%06X · t 0x%04X" % (err, st, p_raw, t_raw))

        if err == 0 and not stub:
            self.lbl_press.configure(text="气压  %.2f kPa" % press_kpa(p_raw),
                                     foreground="#3B6D11")
            self.lbl_temp.configure(text="温度  %.1f ℃" % temp_c(t_raw), foreground="#3B6D11")
            self.log("[采集] err=0x00  st=0x%02X  p=%02X %02X %02X  t=%02X %02X"
                     % (st, regs[2], regs[3], regs[4], regs[5], regs[6]))
            self.status("采集完成：%.2f kPa / %.1f ℃  (%.0f ms)"
                        % (press_kpa(p_raw), temp_c(t_raw), dt), "#3B6D11")
            return

        self.lbl_press.configure(text="气压  —", foreground="#A32D2D")
        self.lbl_temp.configure(text="温度  —", foreground="#A32D2D")

        if err == 0:
            # 新版固件在气压全 00/全 FF 时一定返回 0x02/0x01，走到这里说明板上不是新版
            self.log("[采集] err=0 但数据是平凡值（p=%02X %02X %02X  t=%02X %02X）"
                     % (regs[2], regs[3], regs[4], regs[5], regs[6]))
            self.log("        新版固件在这种情况下会返回 err=0x02/0x01；旧版裸透传因为")
            self.log("        len=cmd[2]=0 直接 return，才会恒为全 0 —— 请重新编译下载固件")
            self.status("固件疑似未更新：重新编译下载后再试", "#A32D2D")
            return

        text, color = ERR_TEXT.get(err, ("未知错误码 0x%02X" % err, "#A32D2D"))
        self.log("[采集] err=0x%02X  %s" % (err, text))
        if err == 0x02:
            self.log("        err=0x02 表示传感器没电：先点 [上电]（0x0207 会设压并送电）")
        self.status("采集失败：err=0x%02X  %s" % (err, text), color)

    # ------------------------------------------------------------
    def _on_close(self):
        try:
            self.mb.close()
        except Exception:
            pass
        self.destroy()


def main():
    if serial is None:
        root = tk.Tk()
        root.withdraw()
        messagebox.showerror("缺少依赖", "没有 pyserial，请执行：pip install pyserial")
        return
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    App().mainloop()


if __name__ == "__main__":
    main()
