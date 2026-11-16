# -*- coding: utf-8 -*-
"""按 FreeModbus 的真实语义（PDU 地址 +1 再交给回调）回归测试协议层。
   不需要真实硬件：FakeSlave 按固件 CmdHelper.c 的分支应答。"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import spi_check_tool as m


def test_crc():
    assert m.add_crc(bytes([1, 3, 0, 0, 0, 1]))[-2:] == b"\x84\x0a"
    assert m.parse_hex("80 06 FF") == bytes([0x80, 6, 0xFF])
    assert m.parse_hex("0x80,0x06,0XFF") == bytes([0x80, 6, 0xFF])
    assert m.parse_hex("8 6 f") == bytes([8, 6, 0x0F])
    print("CRC / 十六进制解析 ok")


class FakeSlave:
    """模拟 app_freertos.c + CmdHelper.c：回调收到 usAddress，和 40001 / 30001 比较"""

    is_open = True

    def __init__(self, support_power=True, ft_err=0):
        self.buf = b""
        self.recvBuf = None
        self.sendBuf = [0] * 120
        self.support_power = support_power
        self.ft_err = ft_err

    def close(self):
        pass

    def reset_input_buffer(self):
        self.buf = b""

    def read(self, n):
        n = min(n, len(self.buf))
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def write(self, frame):
        body = frame[:-2]
        slave, func = body[0], body[1]
        pdu_addr = (body[2] << 8) | body[3]
        usAddress = pdu_addr + 1          # <- FreeModbus 的 usRegAddress++
        qty = (body[4] << 8) | body[5]
        base = 40001 if func == 0x10 else 30001
        if usAddress != base or usAddress + qty > base + 64:
            self.buf += m.add_crc(bytes([slave, func | 0x80, 0x02]))
            return

        if func == 0x10:
            regs = [(body[7 + 2 * i] << 8) | body[8 + 2 * i] for i in range(qty)]
            self.recvBuf = regs
            self.sendBuf = [0] * 120          # 固件里的 memset(sendBuf, 0, ...)
            cmd_type, head = regs[0], regs[1]
            if cmd_type == m.CMD_TYPE_SPI:                       # 6
                n = regs[2]
                self.sendBuf[:n] = regs[3:3 + n]                 # 假 SPI：发什么就收什么
            elif cmd_type == m.CMD_TYPE_SYS and head == m.SYS_GET_VERSION:
                self.sendBuf[0], self.sendBuf[1] = 1, 2
            elif cmd_type == m.CMD_TYPE_SYS and head == m.SYS_SET_POWER:
                self.sendBuf[0] = 1 if self.support_power else 0
            elif cmd_type == m.CMD_TYPE_SYS and head == m.SYS_2200_FT:
                self.sendBuf[0] = self.ft_err
                if self.ft_err == 0:
                    for i, v in enumerate((0x12, 0x34, 0x56)):
                        self.sendBuf[44 + i] = v
                    self.sendBuf[47], self.sendBuf[48] = 0xAB, 0xCD
            resp = bytes([slave, 0x10]) + body[2:6]
        else:
            data = b"".join(bytes([(self.sendBuf[i] >> 8) & 0xFF, self.sendBuf[i] & 0xFF])
                            for i in range(qty))
            resp = bytes([slave, 0x04, len(data)]) + data
        self.buf += m.add_crc(resp)


def make_board(**kw):
    mb = m.ModbusRtuMaster()
    mb.open = lambda *a, **k: None
    mb.ser = FakeSlave(**kw)
    mb.slave = 1
    mb.timeout = 0.5
    return mb, m.Board(mb)


def main():
    test_crc()
    assert m.ADDR_HOLDING == 40000 and m.ADDR_INPUT == 30000
    print("地址常量 ok:", m.ADDR_HOLDING, m.ADDR_INPUT)

    mb, bd = make_board()
    assert bd.get_version() == (1, 2)
    print("读版本 ok -> v1.2")

    assert bd.spi(bytes([0x80, 0x06, 0xFF])) == bytes([0x80, 0x06, 0xFF])
    assert mb.ser.recvBuf == [6, 0, 3, 0x80, 0x06, 0xFF]
    print("SPI 透传 ok, recvBuf =", mb.ser.recvBuf)

    bd.set_vdd(3300)
    assert mb.ser.recvBuf == [1, 0x207, 3300], mb.ser.recvBuf

    assert bd.set_power(True) == 1
    assert mb.ser.recvBuf == [1, 0x208, 1, 0, 0], mb.ser.recvBuf
    print("上电命令 ok, recvBuf =", mb.ser.recvBuf)

    regs = bd.run_ft(0)
    p = (regs[44] << 16) | (regs[45] << 8) | regs[46]
    t = (regs[47] << 8) | regs[48]
    assert regs[0] == 0 and p == 0x123456 and t == 0xABCD
    print("0x206 解析 ok，气压 = 0x%06X 温度 = 0x%04X" % (p, t))

    # 固件没有 0x208 的场合：回 0，界面据此提示“不支持”
    mb2, bd2 = make_board(support_power=False)
    assert bd2.set_power(True) == 0
    print("不支持电源命令时回读 0 ok")

    try:
        mb.write_holding([1, 0x104], addr=40001)
        raise SystemExit("应该报异常")
    except m.ModbusError as e:
        print("错误地址按预期报错：", e)

    print("ALL PASS")


if __name__ == "__main__":
    main()
