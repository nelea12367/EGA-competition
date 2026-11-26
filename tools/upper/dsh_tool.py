# -*- coding: utf-8 -*-
"""
机械臂参数上位机（只做三件事：编译、烧录、改参数）

  - 不控制电机：不连调试器、不接管关节，避免和遥控抢控制权。
  - 改的参数都写在 Core/Inc/vehicle_config.h 里，改完点保存即可。
  - 除了下面这些参数，其它宏请直接编辑 vehicle_config.h（上位机不碰）。

启动：
    python tools/upper/dsh_tool.py
或双击 tools/upper/run.bat
"""

import os
import re
import shutil
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

PROJECT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CONFIG_REL = os.path.join("Core", "Inc", "vehicle_config.h")

BUILD_PS1 = os.path.join(PROJECT, "build.ps1")
FLASH_PS1 = os.path.join(PROJECT, "flash.ps1")

# ---------------------------------------------------------------------------
# 可调参数：宏名 -> (中文名, 单位, 说明)
# 只允许改这里列出来的宏；其余一律手工编辑 vehicle_config.h。
# ---------------------------------------------------------------------------
PARAMS = [
    ("VEHICLE_STEPPER_MANUAL_DPS",   "大臂手动转速", "°/s", "摇杆推到底时的速度"),
    ("VEHICLE_LOWER_ARM_MANUAL_DPS", "小臂手动转速", "°/s", "摇杆推到底时的速度"),
    ("VEHICLE_WRIST_MANUAL_DPS",     "腕部手动转速", "°/s", "摇杆推到底时的速度"),
    ("VEHICLE_CAL_STEPPER_DEG",      "大臂校准角度", "°",   "把“校准那一刻”定义成这个角度"),
    ("VEHICLE_CAL_WRIST_DEG",        "腕部校准角度", "°",   "把“校准那一刻”定义成这个角度"),
]

JOINTS = ["大臂（步进）", "小臂（舵机）", "腕部（2006）"]
POSE_COUNT = 6
POSE_NAMES = ["C 上 + D 上", "C 上 + D 下", "C 中 + D 上",
              "C 中 + D 下", "C 下 + D 上", "C 下 + D 下"]


# ---------------------------------------------------------------------------
# 工具链探测
# ---------------------------------------------------------------------------
def _first_existing(paths):
    for p in paths:
        if p and os.path.isfile(p):
            return p
    return None


def detect_toolchain():
    """找 arm-none-eabi-gcc / openocd / st-flash；找不到返回 None。"""
    import glob

    roots = [os.environ.get("STM32CUBEIDE_ROOT"), r"C:\ST", r"D:\programming",
             r"D:\ST", r"E:\ST"]
    gcc_dirs = []
    for root in roots:
        if not root or not os.path.isdir(root):
            continue
        gcc_dirs += glob.glob(os.path.join(root, "**", "arm-none-eabi-gcc.exe"),
                              recursive=True)

    gcc = _first_existing(gcc_dirs)
    openocd = None
    if gcc:
        base = os.path.dirname(os.path.dirname(os.path.dirname(gcc)))
        openocd = _first_existing(glob.glob(os.path.join(base, "**", "openocd.exe"),
                                            recursive=True))

    stflash = _first_existing([
        r"D:\programming\stlink-tools\stlink-1.9.0-x86_64-w64-mingw32\bin\st-flash.exe",
    ])
    if stflash is None:
        stflash = _first_existing(glob.glob(os.path.join(r"D:\programming", "**",
                                                         "st-flash.exe"), recursive=True))

    return {"gcc": gcc, "openocd": openocd, "stflash": stflash}


# ---------------------------------------------------------------------------
# 配置文件读写
# ---------------------------------------------------------------------------
def config_path():
    return os.path.join(PROJECT, CONFIG_REL)


def read_config():
    with open(config_path(), encoding="utf-8") as f:
        return f.read()


def write_config(text):
    """写回前先备份一份，防止手滑把自己配置弄丢。"""
    path = config_path()
    stamp = time.strftime("%Y%m%d-%H%M%S")
    shutil.copyfile(path, "%s.bak-%s" % (path, stamp))
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(text)


def read_param(text, macro):
    m = re.search(r"#define\s+%s\s+([-+]?[0-9.]+)f?" % macro, text)
    return float(m.group(1)) if m else None


def replace_param(text, macro, value):
    pattern = re.compile(r"(#define\s+%s\s+)([-+]?[0-9.]+f?)" % macro)
    if not pattern.search(text):
        raise KeyError("配置里找不到 %s" % macro)
    return pattern.sub(lambda m: m.group(1) + ("%.1f" % value), text, count=1)


def read_poses(text):
    """读出 VEHICLE_POSE_TABLE 里的 6×3 数值（度）。"""
    block = re.search(r"#define\s+VEHICLE_POSE_TABLE(.*?)\n\}", text, re.S)
    if not block:
        raise ValueError("配置里找不到 VEHICLE_POSE_TABLE")
    rows = re.findall(r"\{([^{}]*)\}", block.group(1))
    poses = []
    for row in rows:
        nums = [float(x) for x in re.findall(r"[-+]?[0-9]*\.?[0-9]+", row)]
        if len(nums) >= 3:
            poses.append(nums[:3])
    return poses


def replace_poses(text, poses):
    """只改每行大括号里的数字，保留注释和对齐。"""
    block_pat = re.compile(r"(#define\s+VEHICLE_POSE_TABLE.*?\n\})", re.S)
    block = block_pat.search(text)
    if not block:
        raise ValueError("配置里找不到 VEHICLE_POSE_TABLE")

    body = block.group(1)
    row_pat = re.compile(r"(\{[^{}]*\})")
    it = iter(poses)

    def sub_row(m):
        try:
            values = next(it)
        except StopIteration:
            return m.group(1)
        cells = ", ".join("%6.1f" % v for v in values)
        return "{ %s }" % cells

    new_body = row_pat.sub(sub_row, body)
    return text[:block.start(1)] + new_body + text[block.end(1):]


# ---------------------------------------------------------------------------
# 界面
# ---------------------------------------------------------------------------
class App:
    def __init__(self, root):
        self.root = root
        self.tools = detect_toolchain()
        self.proc = None
        self.param_vars = {}
        self.pose_vars = []

        root.title("机械臂参数上位机 — EGA-competition")
        root.geometry("900x700")
        self._build_ui()
        self.reload()

    # ---------- 界面 ----------
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")

        self.btn_build = ttk.Button(top, text="① 编译", command=self.do_build)
        self.btn_flash = ttk.Button(top, text="② 烧录", command=self.do_flash)
        self.btn_both = ttk.Button(top, text="★ 一键编译 + 烧录", command=self.do_both)
        self.btn_cancel = ttk.Button(top, text="取消", command=self.do_cancel, state="disabled")
        for b in (self.btn_build, self.btn_flash, self.btn_both, self.btn_cancel):
            b.pack(side="left", padx=(0, 6))

        ttk.Separator(top, orient="vertical").pack(side="left", fill="y", padx=8)
        ttk.Button(top, text="重读配置", command=self.reload).pack(side="left", padx=(0, 6))
        ttk.Button(top, text="打开工程目录", command=self.open_project).pack(side="left")

        info = ttk.Frame(self.root, padding=(8, 0))
        info.pack(fill="x")
        state = [
            "编译器 " + ("√" if self.tools["gcc"] else "×"),
            "st-flash " + ("√" if self.tools["stflash"] else "×"),
            "OpenOCD " + ("√" if self.tools["openocd"] else "×"),
        ]
        ttk.Label(info, text="   ".join(state), foreground="#555").pack(side="left")

        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, padx=8, pady=8)

        # --- 参数页 ---
        page1 = ttk.Frame(nb, padding=12)
        nb.add(page1, text="机械臂参数")

        ttk.Label(page1, text="关节转速和校准角度",
                  font=("Microsoft YaHei UI", 11, "bold")).grid(
            row=0, column=0, columnspan=4, sticky="w", pady=(0, 10))

        for i, (macro, label, unit, tip) in enumerate(PARAMS):
            row = i + 1
            ttk.Label(page1, text=label, width=16).grid(row=row, column=0, sticky="w", pady=4)
            var = tk.StringVar()
            self.param_vars[macro] = var
            ttk.Entry(page1, textvariable=var, width=12, justify="right").grid(
                row=row, column=1, sticky="w")
            ttk.Label(page1, text=unit, width=5).grid(row=row, column=2, sticky="w")
            ttk.Label(page1, text="%s  ·  %s" % (tip, macro),
                      foreground="#777").grid(row=row, column=3, sticky="w", padx=10)

        ttk.Label(page1,
                  text="小臂是绝对舵机（脉宽直接对应角度），不需要校准角度。\n"
                       "校准：开机自动做一次；机械臂模式下 CH1 拨到底保持 1 秒可重新校准。",
                  foreground="#666", justify="left").grid(
            row=len(PARAMS) + 2, column=0, columnspan=4, sticky="w", pady=(16, 0))

        # --- 姿态页 ---
        page2 = ttk.Frame(nb, padding=12)
        nb.add(page2, text="六个姿态（度）")

        ttk.Label(page2, text="姿态号 = C 档位 × 2 + D 档位　（单位：度）",
                  font=("Microsoft YaHei UI", 11, "bold")).grid(
            row=0, column=0, columnspan=4, sticky="w", pady=(0, 10))

        for j, name in enumerate(JOINTS):
            ttk.Label(page2, text=name, width=16).grid(row=1, column=j + 1, pady=(0, 6))

        for i in range(POSE_COUNT):
            ttk.Label(page2, text="姿态%d  %s" % (i, POSE_NAMES[i]), width=18).grid(
                row=i + 2, column=0, sticky="w", pady=3)
            row_vars = []
            for j in range(3):
                var = tk.StringVar()
                row_vars.append(var)
                ttk.Entry(page2, textvariable=var, width=10, justify="right").grid(
                    row=i + 2, column=j + 1, padx=6)
            self.pose_vars.append(row_vars)

        ttk.Label(page2,
                  text="某个关节方向反了，不要在这里改正负号——去改 vehicle_config.h 里的\n"
                       "VEHICLE_STEPPER_DIR_POSITIVE / VEHICLE_WRIST_REVERSED。",
                  foreground="#666", justify="left").grid(
            row=POSE_COUNT + 3, column=0, columnspan=4, sticky="w", pady=(16, 0))

        # --- 底部 ---
        bottom = ttk.Frame(self.root, padding=(8, 0))
        bottom.pack(fill="x")
        ttk.Button(bottom, text="保存到配置文件", command=self.save).pack(side="left")
        ttk.Label(bottom, text="（保存前自动备份 .bak-时间戳）",
                  foreground="#777").pack(side="left", padx=8)

        log_frame = ttk.Frame(self.root, padding=8)
        log_frame.pack(fill="both", expand=False)
        self.log = tk.Text(log_frame, height=11, wrap="word", state="disabled",
                           font=("Consolas", 9))
        self.log.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(log_frame, command=self.log.yview)
        sb.pack(side="right", fill="y")
        self.log.config(yscrollcommand=sb.set)

    # ---------- 日志 ----------
    def _log(self, text):
        self.log.config(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def open_project(self):
        os.startfile(PROJECT)

    # ---------- 读取/保存 ----------
    def reload(self):
        try:
            text = read_config()
        except Exception as exc:                      # noqa: BLE001
            messagebox.showerror("读不到配置", str(exc))
            return

        for macro, _label, _unit, _tip in PARAMS:
            value = read_param(text, macro)
            self.param_vars[macro].set("" if value is None else ("%.1f" % value))

        poses = read_poses(text)
        for i in range(POSE_COUNT):
            for j in range(3):
                if i < len(poses):
                    self.pose_vars[i][j].set("%.1f" % poses[i][j])
                else:
                    self.pose_vars[i][j].set("0.0")

        self._log("已读取 %s" % CONFIG_REL)

    def save(self):
        try:
            text = read_config()

            for macro, label, _unit, _tip in PARAMS:
                raw = self.param_vars[macro].get().strip()
                try:
                    value = float(raw)
                except ValueError:
                    messagebox.showerror("数值不合法", "「%s」不是数字：%s" % (label, raw))
                    return
                text = replace_param(text, macro, value)

            poses = []
            for i in range(POSE_COUNT):
                row = []
                for j in range(3):
                    raw = self.pose_vars[i][j].get().strip()
                    try:
                        row.append(float(raw))
                    except ValueError:
                        messagebox.showerror(
                            "数值不合法",
                            "姿态%d 的「%s」不是数字：%s" % (i, JOINTS[j], raw))
                        return
                poses.append(row)
            text = replace_poses(text, poses)

            write_config(text)
            self._log("已保存（已备份 .bak-时间戳）")
            messagebox.showinfo("已保存", "参数已写入 vehicle_config.h。\n"
                                          "下一步点「一键编译 + 烧录」。")
        except Exception as exc:                      # noqa: BLE001
            messagebox.showerror("保存失败", str(exc))

    # ---------- 编译 / 烧录 ----------
    def _run(self, args, title):
        if self.proc is not None:
            messagebox.showinfo("正在忙", "上一个任务还没结束。")
            return
        self.btn_cancel.config(state="normal")
        self._log("=== %s ===" % title)

        def worker():
            try:
                self.proc = subprocess.Popen(
                    args, cwd=PROJECT, stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                    errors="replace", bufsize=1)
                for line in self.proc.stdout:
                    line = line.rstrip()
                    if line:
                        self.root.after(0, self._log, line)
                code = self.proc.wait()
                self.root.after(0, self._done, title, code)
            except Exception as exc:                  # noqa: BLE001
                self.root.after(0, self._done, title, -1)
                self.root.after(0, self._log, "启动失败: %s" % exc)

        threading.Thread(target=worker, daemon=True).start()

    def _done(self, title, code):
        self.proc = None
        self.btn_cancel.config(state="disabled")
        if code == 0:
            self._log("=== %s 成功 ===" % title)
        else:
            self._log("=== %s 失败（exit=%d）===" % (title, code))

    def do_build(self):
        self._run(["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass",
                   "-File", BUILD_PS1], "编译")

    def do_flash(self):
        self._run(["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass",
                   "-File", FLASH_PS1], "烧录")

    def do_both(self):
        if self.proc is not None:
            messagebox.showinfo("正在忙", "上一个任务还没结束。")
            return
        self.btn_cancel.config(state="normal")

        def worker():
            for title, script in (("编译", BUILD_PS1), ("烧录", FLASH_PS1)):
                self.root.after(0, self._log, "=== %s ===" % title)
                try:
                    p = subprocess.Popen(["powershell", "-NoProfile", "-ExecutionPolicy",
                                          "Bypass", "-File", script],
                                         cwd=PROJECT, stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, text=True,
                                         encoding="utf-8", errors="replace", bufsize=1)
                    self.proc = p
                    for line in p.stdout:
                        line = line.rstrip()
                        if line:
                            self.root.after(0, self._log, line)
                    code = p.wait()
                except Exception as exc:              # noqa: BLE001
                    self.root.after(0, self._log, "启动失败: %s" % exc)
                    code = -1
                self.proc = None
                if code != 0:
                    self.root.after(0, self._log,
                                    "=== %s 失败（exit=%d），已停止 ===" % (title, code))
                    self.root.after(0, lambda: self.btn_cancel.config(state="disabled"))
                    return

            self.root.after(0, self._log, "=== 一键编译+烧录 完成 ===")
            self.root.after(0, lambda: self.btn_cancel.config(state="disabled"))

        threading.Thread(target=worker, daemon=True).start()

    def do_cancel(self):
        if self.proc is not None:
            try:
                self.proc.kill()
                self._log("已取消")
            except Exception:                         # noqa: BLE001
                pass


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
