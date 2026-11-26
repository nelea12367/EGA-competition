# -*- coding: utf-8 -*-
"""
读取固件里的诊断变量（只读，不接管任何控制）。

用法：
    python tools/upper/dbg_read.py            # 读一次
    python tools/upper/dbg_read.py --watch 5  # 每秒读一次，共 5 次
    python tools/upper/dbg_read.py --clear    # 先把峰值清 0，再读

原理：st-util 起一个 GDB server，用 GDB remote 协议 halt -> 读内存 -> continue。
符号地址直接从 build\DM_MC02_Chassis.elf 的符号表拿。
"""

import os
import re
import socket
import struct
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ELF = os.path.join(ROOT, "build", "DM_MC02_Chassis.elf")
ST_UTIL_CANDIDATES = [
    r"D:\programming\stlink-tools\stlink-1.9.0-x86_64-w64-mingw32\bin\st-util.exe",
]
NM_CANDIDATES = [
    r"D:\programming\STM32CubeIDE_2.3.0\STM32CubeIDE\plugins"
    r"\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32"
    r".14.3.rel1.win32_1.0.100.202602081740\tools\bin\arm-none-eabi-nm.exe",
]

WATCH = [
    ("g_dbg_mode",        "<i", "机械臂模式(1=是)"),
    ("g_dbg_stick_step",  "<f", "CH3 摇杆量(-1~1)"),
    ("g_dbg_step_ref",    "<i", "步进零点已建立(1=是)"),
    ("g_dbg_step_hz_now", "<f", "当前脉冲频率(Hz)"),
    ("g_dbg_step_hz_max", "<f", "见过的最大频率(Hz)"),
    ("g_dbg_step_running", "<i", "定时器在发脉冲(1=是)"),
    ("g_dbg_step_pulse",  "<i", "脉冲计数"),
    ("g_dbg_pulses_total", "<I", "累计发过的脉冲总数"),
]


def find_first(paths, pattern=None):
    for p in paths:
        if os.path.isfile(p):
            return p
    if pattern:
        import glob
        hit = glob.glob(pattern, recursive=True)
        if hit:
            return hit[0]
    return None


def load_symbols(names):
    nm = find_first(NM_CANDIDATES,
                    r"D:\programming\**\arm-none-eabi-nm.exe")
    if not nm:
        raise RuntimeError("找不到 arm-none-eabi-nm.exe")
    out = subprocess.run([nm, ELF], capture_output=True, text=True,
                         encoding="utf-8", errors="replace").stdout
    table = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3:
            try:
                table[parts[2]] = int(parts[0], 16)
            except ValueError:
                pass
    missing = [n for n in names if n not in table]
    if missing:
        raise RuntimeError("固件里找不到符号: %s" % ", ".join(missing))
    return {n: table[n] for n in names}


class Rsp:
    def __init__(self, port):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=10)
        self.sock.settimeout(4.0)

    def _recv(self):
        data = b""
        while True:
            ch = self.sock.recv(1)
            if not ch:
                raise RuntimeError("连接被关闭")
            data += ch
            if data.startswith(b"+"):
                data = data[1:]
                continue
            if data.startswith(b"$"):
                if data.endswith(b"#"):
                    self.sock.recv(2)
                    return data[1:-1].decode()
                continue
            if data.startswith(b"-"):
                data = data[1:]
                continue

    def cmd(self, body):
        packet = ("$%s#%02x" % (body, sum(body.encode()) & 0xFF)).encode()
        self.sock.sendall(packet)
        reply = self._recv()
        self.sock.sendall(b"+")
        return reply

    def read_mem(self, addr, size):
        return bytes.fromhex(self.cmd("m%x,%x" % (addr, size)))

    def write_mem(self, addr, data):
        return self.cmd("M%x,%x:%s" % (addr, len(data), data.hex()))

    def halt(self):
        self.sock.settimeout(1.0)
        self.sock.sendall(b"\x03")
        try:
            self._recv()
        except (socket.timeout, TimeoutError):
            pass
        finally:
            self.sock.settimeout(4.0)

    def cont(self):
        self.sock.sendall(b"$c#63")
        self.sock.settimeout(0.1)
        try:
            self.sock.recv(64)
        except (socket.timeout, TimeoutError, OSError):
            pass
        finally:
            self.sock.settimeout(4.0)


def start_stutil():
    exe = find_first(ST_UTIL_CANDIDATES, r"D:\programming\**\st-util.exe")
    if not exe:
        raise RuntimeError("找不到 st-util.exe")
    for port in range(4242, 4270):
        flags = 0x08000000 if os.name == "nt" else 0
        proc = subprocess.Popen([exe, "--listen_port", str(port)],
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL,
                                creationflags=flags)
        for _ in range(20):
            try:
                return proc, Rsp(port)
            except OSError:
                time.sleep(0.15)
        proc.kill()
    raise RuntimeError("st-util 起不来（端口都被占？）")


def main():
    argv = sys.argv[1:]
    count = 1
    if "--watch" in argv:
        count = int(argv[argv.index("--watch") + 1])

    syms = load_symbols([n for n, _f, _d in WATCH])
    proc, rsp = start_stutil()
    try:
        if "--clear" in argv:
            rsp.halt()
            rsp.write_mem(syms["g_dbg_step_hz_max"], struct.pack("<f", 0.0))
            rsp.cont()
            print("峰值已清零")

        for i in range(count):
            rsp.halt()
            values = {}
            for name, fmt, _desc in WATCH:
                raw = rsp.read_mem(syms[name], struct.calcsize(fmt))
                values[name] = struct.unpack(fmt, raw)[0]
            rsp.cont()

            print("--- 第 %d 次 (t=%.1fs) ---" % (i + 1, time.time() % 1000))
            for name, _fmt, desc in WATCH:
                v = values[name]
                if isinstance(v, float):
                    print("  %-20s = %10.2f   %s" % (name, v, desc))
                else:
                    print("  %-20s = %10d   %s" % (name, v, desc))
            if i + 1 < count:
                time.sleep(1.0)
    finally:
        try:
            rsp.cont()
        except Exception:
            pass
        try:
            proc.kill()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
