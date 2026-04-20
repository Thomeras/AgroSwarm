#!/usr/bin/env python3
"""
isaac_launcher.py — Interactive terminal launcher for Isaac Sim + Pegasus + PX4

FLOW:
  Step 1  — Launch mode  (Isaac only / + PX4 / + PX4 + ROS2 bridge)
  Step 2  — Isaac Sim mode  (GUI / Headless)
  Step 3  — Launch sequence  (PX4 SITL → wait → Isaac Sim → optional DDS bridge)

Usage:
  python3 isaac_launcher.py
  ./isaac_launcher.py
"""

import curses
import json
import locale
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

# ── Paths ─────────────────────────────────────────────────────────────────────
WS_DIR        = "/home/tj/_Data/_Projekty/TJlabs/scout_ws"
PX4_DIR       = os.path.expanduser("~/PX4-Autopilot")
ISAAC_VENV    = os.path.expanduser("~/isaac_env_511")
CONFIG_FILE   = os.path.expanduser("~/.isaac_launcher_config.json")
ROS2_SETUP    = "/opt/ros/jazzy/setup.bash"
WS_SETUP      = f"{WS_DIR}/install/setup.bash"
SWARM_CENTER_DIR = f"{WS_DIR}/swarm_center"

# PX4 model for Pegasus (MAVLink TCP 4560, autostart 10015)
PX4_MODEL  = "gazebo-classic_iris"
PX4_BIN    = f"{PX4_DIR}/build/px4_sitl_default/bin/px4"
PX4_ROMFS  = f"{PX4_DIR}/ROMFS/px4fmu_common/"
PX4_INIT   = "ROMFS/px4fmu_common/init.d-posix/rcS"

# Isaac Sim launch command (source venv, then isaacsim)
ISAAC_CMD  = f"source {ISAAC_VENV}/bin/activate && isaacsim"
ISAAC_CMD_HEADLESS = f"source {ISAAC_VENV}/bin/activate && isaacsim --headless"

# ── ANSI colours ──────────────────────────────────────────────────────────────
RST  = "\033[0m"
BOLD = "\033[1m"
DIM  = "\033[2m"
GRN  = "\033[92m"
YLW  = "\033[93m"
RED  = "\033[91m"
CYN  = "\033[96m"
BLU  = "\033[94m"
GRY  = "\033[90m"
MGN  = "\033[95m"

def _step(n: int, t: int, msg: str) -> str:
    return f"{MGN}[{n}/{t}]{RST} {msg}"

def _ok(msg: str)  -> str: return f"{GRN}✓{RST}  {msg}"
def _err(msg: str) -> str: return f"{RED}✗{RST}  {msg}"
def _dim(msg: str) -> str: return f"{GRY}   {msg}{RST}"


# ── Child process registry ────────────────────────────────────────────────────
_children: list[subprocess.Popen] = []

def _register(p: subprocess.Popen) -> subprocess.Popen:
    _children.append(p)
    return p

def _kill_all() -> None:
    for p in _children:
        try:
            if p.poll() is None:
                p.terminate()
        except Exception:
            pass


# ── Config ────────────────────────────────────────────────────────────────────
def _load_cfg() -> dict:
    try:
        with open(CONFIG_FILE) as f:
            return json.load(f)
    except Exception:
        return {}

def _save_cfg(data: dict) -> None:
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump(data, f, indent=2)
    except Exception:
        pass


# ── Terminal launcher ─────────────────────────────────────────────────────────
def _find_terminal() -> str:
    for t in ("gnome-terminal", "konsole", "xfce4-terminal", "xterm"):
        result = subprocess.run(["which", t], capture_output=True)
        if result.returncode == 0:
            return t
    raise RuntimeError("No terminal emulator found. Install xterm: sudo apt install xterm")

_TERMINAL: Optional[str] = None

def _open_terminal(title: str, cmd: str) -> subprocess.Popen:
    global _TERMINAL
    if _TERMINAL is None:
        _TERMINAL = _find_terminal()

    bash_cmd = f"{cmd}; exec bash"

    if _TERMINAL == "gnome-terminal":
        args = ["gnome-terminal", f"--title={title}", "--",
                "bash", "-c", bash_cmd]
    elif _TERMINAL == "konsole":
        args = ["konsole", f"--title={title}",
                "-e", "bash", "-c", bash_cmd]
    elif _TERMINAL == "xfce4-terminal":
        args = ["xfce4-terminal", f"--title={title}",
                "-e", f"bash -c {bash_cmd!r}"]
    else:  # xterm
        args = ["xterm",
                "-title", title,
                "-fa", "Monospace", "-fs", "10",
                "-tn", "xterm-256color",
                "-geometry", "140x40",
                "-e", "bash", "-c", bash_cmd]

    env = os.environ.copy()
    if "DISPLAY" not in env:
        env["DISPLAY"] = ":0"

    return _register(subprocess.Popen(args, env=env))


# ── Curses colour setup ───────────────────────────────────────────────────────
def _setup_colors() -> None:
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_WHITE,   -1)
    curses.init_pair(2, curses.COLOR_BLACK,   curses.COLOR_MAGENTA)
    curses.init_pair(3, curses.COLOR_YELLOW,  -1)
    curses.init_pair(4, curses.COLOR_GREEN,   -1)
    curses.init_pair(5, curses.COLOR_RED,     -1)
    curses.init_pair(6, curses.COLOR_WHITE,   -1)
    curses.init_pair(7, curses.COLOR_MAGENTA, -1)


def _draw_border(win, title: str) -> None:
    win.box()
    h, w = win.getmaxyx()
    label = f"  {title}  "
    win.addstr(0, max(2, (w - len(label)) // 2), label,
               curses.color_pair(3) | curses.A_BOLD)


def _draw_footer(win, text: str) -> None:
    h, w = win.getmaxyx()
    try:
        win.addstr(h - 2, 2, text[:w - 4], curses.color_pair(6) | curses.A_DIM)
    except curses.error:
        pass


def _draw_row(win, y: int, w: int, text: str, selected: bool) -> None:
    try:
        if selected:
            bar = f"  > {text:<{max(1, w - 8)}} "
            win.addstr(y, 3, bar[:w - 4], curses.color_pair(2) | curses.A_BOLD)
        else:
            win.addstr(y, 3, f"    {text}"[:w - 4], curses.color_pair(1))
    except curses.error:
        pass


# ── Screen: Launch mode ───────────────────────────────────────────────────────
_LAUNCH_MODES: list[tuple[str, str]] = [
    ("isaac_only",   "Isaac Sim only          — GUI, no PX4"),
    ("isaac_px4",    "Isaac Sim + PX4 SITL    — MAVLink TCP 4560, Pegasus"),
    ("isaac_px4_ros","Isaac Sim + PX4 + ROS2  — MAVLink + MicroXRCE bridge"),
    ("isaac_e2e",    "Isaac Full E2E Mission  — PX4 + Isaac + ROS2 mission + GCS"),
]

def _screen_launch_mode(stdscr, default: str) -> Optional[str]:
    _setup_colors()
    curses.curs_set(0)
    stdscr.keypad(True)

    keys = [m[0] for m in _LAUNCH_MODES]
    sel  = keys.index(default) if default in keys else 1  # default: isaac_px4

    while True:
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        _draw_border(stdscr, "Isaac Launcher  —  Launch Mode")

        try:
            stdscr.addstr(2, 4, "Choose launch mode:", curses.color_pair(7))
        except curses.error:
            pass

        for i, (key, label) in enumerate(_LAUNCH_MODES):
            _draw_row(stdscr, 4 + i, w, label, i == sel)

        # Info panel
        try:
            stdscr.addstr(h - 5, 2, "─" * min(w - 4, 72), curses.color_pair(6) | curses.A_DIM)
            info = {
                "isaac_only":    "Isaac Sim GUI spustí se v novém terminálu. PX4 nespouštíme.",
                "isaac_px4":     "PX4 SITL → 20s čekání → Isaac Sim. MAVLink TCP 4560.",
                "isaac_px4_ros": "PX4 SITL → Isaac Sim → MicroXRCE bridge (port 8888) pro ROS2.",
                "isaac_e2e":     "PX4 SITL → Isaac Sim → MicroXRCE → ROS2 E2E mise → manual_controller → Swarm Center.",
            }
            stdscr.addstr(h - 4, 4, info[_LAUNCH_MODES[sel][0]][:w - 8],
                          curses.color_pair(6) | curses.A_DIM)
        except curses.error:
            pass

        _draw_footer(stdscr, "UP/DOWN  Navigate    ENTER  Confirm    Q  Quit")
        stdscr.refresh()

        key = stdscr.getch()
        if key == curses.KEY_UP and sel > 0:
            sel -= 1
        elif key == curses.KEY_DOWN and sel < len(_LAUNCH_MODES) - 1:
            sel += 1
        elif key in (curses.KEY_ENTER, 10, 13):
            return _LAUNCH_MODES[sel][0]
        elif key in (ord("q"), ord("Q"), 27):
            return None


# ── Screen: Isaac mode (GUI / Headless) ──────────────────────────────────────
_ISAAC_MODES: list[tuple[str, str]] = [
    ("gui",      "GUI mode      — otevře grafické okno Isaac Sim"),
    ("headless", "Headless mode — bez grafiky, vhodné pro ML training"),
]

def _screen_isaac_mode(stdscr, default: str) -> Optional[str]:
    _setup_colors()
    curses.curs_set(0)
    stdscr.keypad(True)

    keys = [m[0] for m in _ISAAC_MODES]
    sel  = keys.index(default) if default in keys else 0

    while True:
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        _draw_border(stdscr, "Isaac Launcher  —  Isaac Sim Mode")

        try:
            stdscr.addstr(2, 4, "Jak spustit Isaac Sim?", curses.color_pair(7))
        except curses.error:
            pass

        for i, (key, label) in enumerate(_ISAAC_MODES):
            _draw_row(stdscr, 4 + i, w, label, i == sel)

        # Show the actual command
        cmd = ISAAC_CMD if sel == 0 else ISAAC_CMD_HEADLESS
        try:
            stdscr.addstr(h - 5, 2, "─" * min(w - 4, 72), curses.color_pair(6) | curses.A_DIM)
            stdscr.addstr(h - 4, 4, f"$ {cmd}"[:w - 8], curses.color_pair(7) | curses.A_DIM)
        except curses.error:
            pass

        _draw_footer(stdscr, "UP/DOWN  Navigate    ENTER  Confirm    Q  Quit")
        stdscr.refresh()

        key = stdscr.getch()
        if key == curses.KEY_UP and sel > 0:
            sel -= 1
        elif key == curses.KEY_DOWN and sel < len(_ISAAC_MODES) - 1:
            sel += 1
        elif key in (curses.KEY_ENTER, 10, 13):
            return _ISAAC_MODES[sel][0]
        elif key in (ord("q"), ord("Q"), 27):
            return None


# ── Step 2: Launch sequences ──────────────────────────────────────────────────
def _kill_stale_px4() -> None:
    print(f"{YLW}[CLEANUP]{RST} Killing stale PX4 processes...")
    subprocess.run(
        'pkill -9 -f "px4" 2>/dev/null; '
        'pkill -9 -f "MicroXRCEAgent" 2>/dev/null',
        shell=True, executable="/bin/bash",
    )
    time.sleep(1)
    print(f"{GRN}[CLEANUP]{RST} Done.\n")


def _countdown(n: int, t: int, label: str, secs: int) -> None:
    for i in range(secs, 0, -1):
        sys.stdout.write(f"\r{_step(n, t, f'{label} ({i}s)...')}   ")
        sys.stdout.flush()
        time.sleep(1)
    sys.stdout.write(f"\r{_step(n, t, label + ' — done')}            \n")
    sys.stdout.flush()


def _launch_isaac_only(headless: bool) -> None:
    print(f"\n{BOLD}{BLU}{'━' * 50}{RST}")
    print(f"{BOLD}{BLU}  Isaac Sim — standalone{RST}")
    print(f"{BOLD}{BLU}{'━' * 50}{RST}\n")

    T = 1
    isaac_cmd = ISAAC_CMD_HEADLESS if headless else ISAAC_CMD
    mode_label = "headless" if headless else "GUI"
    print(_step(1, T, f"Spouštím Isaac Sim ({mode_label})..."))
    _open_terminal("Isaac Sim", isaac_cmd)
    print(_dim("terminál otevřen"))
    print(f"\n{GRN}{BOLD}Isaac Sim spuštěn!{RST}")
    _print_isaac_hint()


def _launch_isaac_px4(headless: bool) -> None:
    print(f"\n{BOLD}{BLU}{'━' * 50}{RST}")
    print(f"{BOLD}{BLU}  Isaac Sim + PX4 SITL{RST}")
    print(f"{BOLD}{BLU}{'━' * 50}{RST}\n")

    _kill_stale_px4()

    T = 3
    isaac_cmd = ISAAC_CMD_HEADLESS if headless else ISAAC_CMD

    # 1/3  PX4 SITL
    print(_step(1, T, f"Spouštím PX4 SITL (model: {CYN}{PX4_MODEL}{RST})..."))
    _open_terminal(
        "PX4 SITL — Pegasus",
        f"cd {PX4_DIR} && PX4_SIM_MODEL={PX4_MODEL} "
        f"{PX4_BIN} {PX4_ROMFS} -s {PX4_INIT}",
    )
    print(_dim("Čekat na: INFO [commander] Ready for takeoff!"))

    # 2/3  Wait for PX4
    _countdown(2, T, "Čekám na PX4 inicializaci", 20)

    # 3/3  Isaac Sim
    mode_label = "headless" if headless else "GUI"
    print(_step(3, T, f"Spouštím Isaac Sim ({mode_label})..."))
    _open_terminal("Isaac Sim", isaac_cmd)
    print(_dim("terminál otevřen"))

    print(f"\n{GRN}{BOLD}PX4 + Isaac Sim spuštěny!{RST}")
    _print_px4_hint()
    _print_isaac_hint()


def _launch_isaac_px4_ros(headless: bool) -> None:
    print(f"\n{BOLD}{BLU}{'━' * 50}{RST}")
    print(f"{BOLD}{BLU}  Isaac Sim + PX4 SITL + MicroXRCE (ROS2){RST}")
    print(f"{BOLD}{BLU}{'━' * 50}{RST}\n")

    _kill_stale_px4()

    T = 5
    isaac_cmd = ISAAC_CMD_HEADLESS if headless else ISAAC_CMD

    # 1/5  PX4 SITL
    print(_step(1, T, f"Spouštím PX4 SITL (model: {CYN}{PX4_MODEL}{RST})..."))
    _open_terminal(
        "PX4 SITL — Pegasus",
        f"cd {PX4_DIR} && PX4_SIM_MODEL={PX4_MODEL} "
        f"{PX4_BIN} {PX4_ROMFS} -s {PX4_INIT}",
    )
    print(_dim("Čekat na: INFO [commander] Ready for takeoff!"))

    # 2/5  Wait for PX4
    _countdown(2, T, "Čekám na PX4 inicializaci", 20)

    # 3/5  Isaac Sim
    mode_label = "headless" if headless else "GUI"
    print(_step(3, T, f"Spouštím Isaac Sim ({mode_label})..."))
    _open_terminal("Isaac Sim", isaac_cmd)
    print(_dim("terminál otevřen"))

    # 4/5  Wait for Isaac Sim to start
    _countdown(4, T, "Čekám na Isaac Sim", 15)

    # 5/5  MicroXRCE-DDS bridge
    print(_step(5, T, "Spouštím MicroXRCE-DDS bridge (port 8888)..."))
    _open_terminal("MicroXRCE-DDS", "MicroXRCEAgent udp4 -p 8888")
    print(_dim("terminál otevřen"))

    print(f"\n{GRN}{BOLD}Isaac Sim + PX4 + ROS2 bridge spuštěny!{RST}")
    _print_px4_hint()
    _print_isaac_hint()
    print(f"{GRY}  ROS2 bridge:     MicroXRCEAgent port 8888{RST}")
    print(f"{YLW}  Nezapomeňte:     source /opt/ros/jazzy/setup.bash{RST}")


def _ros2_env_prefix() -> str:
    return f"source {ROS2_SETUP} && source {WS_SETUP}"


def _launch_isaac_e2e(headless: bool) -> None:
    print(f"\n{BOLD}{BLU}{'━' * 50}{RST}")
    print(f"{BOLD}{BLU}  Isaac Full E2E Mission{RST}")
    print(f"{BOLD}{BLU}{'━' * 50}{RST}\n")

    _kill_stale_px4()

    T = 8
    isaac_cmd = ISAAC_CMD_HEADLESS if headless else ISAAC_CMD
    ros_env = _ros2_env_prefix()

    print(_step(1, T, f"Spouštím PX4 SITL (model: {CYN}{PX4_MODEL}{RST})..."))
    _open_terminal(
        "PX4 SITL — Pegasus",
        f"cd {PX4_DIR} && PX4_SIM_MODEL={PX4_MODEL} "
        f"{PX4_BIN} {PX4_ROMFS} -s {PX4_INIT}",
    )
    print(_dim("Čekat na: INFO [commander] Ready for takeoff!"))

    _countdown(2, T, "Čekám na PX4 inicializaci", 20)

    mode_label = "headless" if headless else "GUI"
    print(_step(3, T, f"Spouštím Isaac Sim ({mode_label})..."))
    _open_terminal("Isaac Sim", isaac_cmd)
    print(_dim("terminál otevřen"))

    _countdown(4, T, "Čekám na Isaac Sim", 15)

    print(_step(5, T, "Spouštím MicroXRCE-DDS bridge (port 8888)..."))
    _open_terminal("MicroXRCE-DDS", "MicroXRCEAgent udp4 -p 8888")
    print(_dim("terminál otevřen"))

    _countdown(6, T, "Čekám na DDS bridge", 3)

    print(_step(7, T, "Spouštím ROS2 E2E mission backend..."))
    _open_terminal(
        "ROS2 — Isaac E2E Mission",
        f"cd {WS_DIR} && {ros_env} && ros2 launch scout_control isaac_e2e_mission.launch.py",
    )
    print(_dim("launch terminál otevřen"))

    print(_step(8, T, "Spouštím manual_controller a Swarm Center..."))
    _open_terminal(
        "manual_controller",
        f"cd {WS_DIR} && {ros_env} && ros2 run scout_control manual_controller",
    )
    _open_terminal(
        "Swarm Center",
        f"cd {SWARM_CENTER_DIR} && python3 main.py --drones 1 --base-port 14540 "
        f"--world-image ../worlds/agro_field_overlay.png",
    )
    print(_dim("extra terminály otevřeny"))

    print(f"\n{GRN}{BOLD}Isaac Full E2E stack spuštěn!{RST}")
    _print_px4_hint()
    _print_isaac_hint()
    print(f"{GRY}  ROS2 launch:      isaac_e2e_mission.launch.py{RST}")
    print(f"{GRY}  Manual control:   manual_controller{RST}")
    print(f"{GRY}  GCS:              Swarm Center (1 drone, overlay map){RST}")
    print(f"{YLW}  Další krok:       V Isaac Sim otevři agro_field.usd, nahraj Iris a dej Play ▶{RST}")


def _print_px4_hint() -> None:
    print(f"{GRY}  PX4 model:       {PX4_MODEL}  (MAVLink TCP 4560){RST}")
    print(f"{GRY}  Test příkazy:    commander takeoff / commander land{RST}")

def _print_isaac_hint() -> None:
    print(f"{GRY}  Isaac Sim:       Pegasus tab → Load Scene → Iris → Load Vehicle → Play ▶{RST}")
    print(f"{GRY}  Venv:            {ISAAC_VENV}{RST}")


# ── Preflight checks ──────────────────────────────────────────────────────────
def _check_venv() -> bool:
    activate = Path(ISAAC_VENV) / "bin" / "activate"
    if activate.exists():
        print(f"{_ok(f'Isaac venv nalezen: {CYN}{ISAAC_VENV}{RST}')}")
        return True
    print(f"{_err(f'Isaac venv NENALEZEN: {ISAAC_VENV}')}")
    print(f"  Nainstaluj Isaac Sim do:  {ISAAC_VENV}\n")
    return False

def _check_px4_bin() -> bool:
    if Path(PX4_BIN).exists():
        print(f"{_ok(f'PX4 binary: {CYN}{PX4_BIN}{RST}')}")
        return True
    print(f"{YLW}[WARN]{RST} PX4 binary nenalezen: {PX4_BIN}")
    print(f"       Spusť nejdřív:  cd {PX4_DIR} && make px4_sitl_default\n")
    return False


def _check_ws_setup() -> bool:
    if Path(WS_SETUP).exists():
        print(f"{_ok(f'ROS2 workspace setup: {CYN}{WS_SETUP}{RST}')}")
        return True
    print(f"{YLW}[WARN]{RST} Workspace setup nenalezen: {WS_SETUP}")
    print(f"       Spusť nejdřív:  cd {WS_DIR} && colcon build --packages-select scout_control\n")
    return False


def _check_swarm_center() -> bool:
    if Path(SWARM_CENTER_DIR).exists():
        print(f"{_ok(f'Swarm Center dir: {CYN}{SWARM_CENTER_DIR}{RST}')}")
        return True
    print(f"{YLW}[WARN]{RST} Swarm Center dir nenalezen: {SWARM_CENTER_DIR}\n")
    return False


# ── Signal handler ────────────────────────────────────────────────────────────
def _on_sigint(sig, frame) -> None:
    try:
        curses.endwin()
    except Exception:
        pass
    print(f"\n{YLW}Shutting down...{RST}")
    _kill_all()
    sys.exit(0)


# ── Header banner ─────────────────────────────────────────────────────────────
_BANNER = f"""\
{MGN}{BOLD}
  ╔══════════════════════════════════════╗
  ║       Isaac Launcher                 ║
  ║   Isaac Sim · Pegasus · PX4 SITL    ║
  ╚══════════════════════════════════════╝
{RST}"""


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    locale.setlocale(locale.LC_ALL, "")
    signal.signal(signal.SIGINT, _on_sigint)

    cfg           = _load_cfg()
    last_mode     = cfg.get("launch_mode", "isaac_px4")
    last_isaac    = cfg.get("isaac_mode",  "gui")

    os.system("clear")
    print(_BANNER)

    # ── Preflight ─────────────────────────────────────────────────────────────
    venv_ok = _check_venv()
    px4_ok  = _check_px4_bin()
    ws_ok   = _check_ws_setup()
    gcs_ok  = _check_swarm_center()
    print()

    if not venv_ok:
        print(f"{RED}Venv nenalezen — nelze pokračovat.{RST}")
        sys.exit(1)

    # ── Step 1: Launch mode ───────────────────────────────────────────────────
    launch_mode = curses.wrapper(_screen_launch_mode, last_mode)
    if launch_mode is None:
        print("Aborted.")
        sys.exit(0)

    cfg["launch_mode"] = launch_mode
    _save_cfg(cfg)
    label = next(l for k, l in _LAUNCH_MODES if k == launch_mode)
    print(f"{_ok(f'Mode: {CYN}{label.strip()}{RST}')}\n")

    # Warn if PX4 needed but missing
    if launch_mode in ("isaac_px4", "isaac_px4_ros", "isaac_e2e") and not px4_ok:
        print(f"{YLW}[WARN]{RST} PX4 binary chybí — launcher pokračuje, ale PX4 terminál selže.\n")
    if launch_mode == "isaac_e2e" and not ws_ok:
        print(f"{YLW}[WARN]{RST} ROS2 workspace není buildnutý — E2E launch terminál pravděpodobně selže.\n")
    if launch_mode == "isaac_e2e" and not gcs_ok:
        print(f"{YLW}[WARN]{RST} Swarm Center adresář chybí — GCS terminál pravděpodobně selže.\n")

    # ── Step 2: Isaac mode ────────────────────────────────────────────────────
    isaac_mode = curses.wrapper(_screen_isaac_mode, last_isaac)
    if isaac_mode is None:
        print("Aborted.")
        sys.exit(0)

    cfg["isaac_mode"] = isaac_mode
    _save_cfg(cfg)
    headless = (isaac_mode == "headless")
    print(f"{_ok(f'Isaac Sim mode: {CYN}{isaac_mode}{RST}')}\n")

    # ── Step 3: Launch ────────────────────────────────────────────────────────
    if launch_mode == "isaac_only":
        _launch_isaac_only(headless)
    elif launch_mode == "isaac_px4":
        _launch_isaac_px4(headless)
    elif launch_mode == "isaac_px4_ros":
        _launch_isaac_px4_ros(headless)
    else:
        _launch_isaac_e2e(headless)

    # ── Done — wait ───────────────────────────────────────────────────────────
    print(f"\n{GRY}Terminály běží. Stiskni ENTER pro ukončení launcheru (terminály zůstanou otevřené)...{RST}")
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        pass

    print(f"{YLW}Isaac Launcher ukončen.{RST}")


if __name__ == "__main__":
    main()
