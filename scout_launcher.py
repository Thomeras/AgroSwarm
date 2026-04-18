#!/usr/bin/env python3
"""
scout_launcher.py — Interactive terminal launcher for ROS2 / PX4 / Gazebo

Replaces the manual 4-terminal setup with a guided curses UI.

FLOW:
  Step 1  — World selection  (curses list)
  Step 2  — Auto launch sequence  (PX4, DDS bridge, QGroundControl)
  Step 3  — Scenario selection  (curses list, R to open reset menu)
  Step 4  — Build scout_control + run scenario in new terminal

Usage:
  python3 scout_launcher.py
  ./scout_launcher.py
"""

import curses
import json
import locale
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml

# ── Paths ─────────────────────────────────────────────────────────────────────
WS_DIR        = "/home/tj/_Data/_Projekty/TJlabs/scout_ws"
PX4_DIR       = os.path.expanduser("~/PX4-Autopilot")
QGC_PATH      = os.path.expanduser("~/QGroundControl-x86_64.AppImage")
RESET_SH      = f"{WS_DIR}/reset.sh"
ROS2_SETUP    = "/opt/ros/jazzy/setup.bash"
WS_SETUP      = f"{WS_DIR}/install/setup.bash"
WORLDS_DIR    = os.path.expanduser("~/PX4-Autopilot/Tools/simulation/gz/worlds")
SCENARIOS_DIR = f"{WS_DIR}/scenarios"
CONFIG_FILE   = os.path.expanduser("~/.scout_launcher_config.json")
DEFAULT_WORLD  = "agricultural_field"
DEFAULT_MODEL  = "gz_x500_mono_cam"

# Available drone models (make target name → human label)
DRONE_MODELS: list[tuple[str, str]] = [
    ("gz_x500_scout",               "x500 + 2-axis gimbal (OakD-Lite) + downward lidar"),
    ("gz_x500_mono_cam_down_lidar", "x500 + downward camera + downward lidar  ← E2E mise, terrain following"),
    ("gz_x500_mono_cam_down",       "x500 + downward camera  ← mapování pole, crosshair"),
    ("gz_x500_mono_cam",            "x500 + forward camera"),
    ("gz_x500_mono_cam_lidar",      "x500 + forward camera + downward lidar"),
    ("gz_x500",                     "x500  (base, no sensors)"),
    ("gz_x500_lidar_down",          "x500 + downward lidar only"),
]

# ── ANSI colours (plain-terminal output) ──────────────────────────────────────
RST  = "\033[0m"
BOLD = "\033[1m"
DIM  = "\033[2m"
GRN  = "\033[92m"
YLW  = "\033[93m"
RED  = "\033[91m"
CYN  = "\033[96m"
BLU  = "\033[94m"
GRY  = "\033[90m"

def _step(n: int, t: int, msg: str) -> str:
    return f"{CYN}[{n}/{t}]{RST} {msg}"

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


# ── Scenario model ────────────────────────────────────────────────────────────
@dataclass
class Scenario:
    name:                   str
    description:            str
    ros2_command:           str
    extra_terminal_commands: list  # optional extra terminals (e.g. curses UI nodes)


def _scan_worlds() -> list[str]:
    p = Path(WORLDS_DIR)
    if not p.exists():
        return [DEFAULT_WORLD]
    # Accept both .sdf and .world — PX4 supports both; use stem as world name
    stems = sorted({x.stem for x in p.glob("*.sdf")} | {x.stem for x in p.glob("*.world")})
    if DEFAULT_WORLD not in stems:
        stems.insert(0, DEFAULT_WORLD)
    return stems or [DEFAULT_WORLD]


def _scan_scenarios() -> list[Scenario]:
    sdir = Path(SCENARIOS_DIR)
    if not sdir.exists():
        return []
    result = []
    for f in sorted(sdir.glob("*.yaml")):
        try:
            data = yaml.safe_load(f.read_text()) or {}
            if "name" in data and "ros2_command" in data:
                result.append(Scenario(
                    name=str(data["name"]),
                    description=str(data.get("description", "")),
                    ros2_command=str(data["ros2_command"]),
                    extra_terminal_commands=list(data.get("extra_terminal_commands", [])),
                ))
        except Exception:
            pass
    return result


# ── Terminal launcher ─────────────────────────────────────────────────────────
def _find_terminal() -> str:
    """Return the first available terminal emulator."""
    for t in ("gnome-terminal", "konsole", "xfce4-terminal", "xterm"):
        result = subprocess.run(["which", t], capture_output=True)
        if result.returncode == 0:
            return t
    raise RuntimeError("No terminal emulator found. Install xterm: sudo apt install xterm")

_TERMINAL: Optional[str] = None

def _open_terminal(title: str, cmd: str) -> subprocess.Popen:
    """Open a new terminal window running cmd; window stays open after exit.

    NOTE: xterm -e requires bash/-c/cmd as *separate* list elements, not one string.
    """
    global _TERMINAL
    if _TERMINAL is None:
        _TERMINAL = _find_terminal()

    # bash wrapper: run cmd, then drop into interactive shell so window stays open
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
    else:  # xterm — fallback
        # -tn xterm-256color: set TERM inside xterm so curses colour pairs work
        # correctly (default xterm TERM is plain "xterm" which lacks 256-colour
        # support and can cause palette rendering bugs in curses UIs like
        # manual_controller).
        # -geometry 140x40: slightly wider/taller than before so the curses UI
        # has room; PX4 / ROS2 terminals benefit from the extra columns too.
        args = ["xterm",
                "-title", title,
                "-fa", "Monospace", "-fs", "10",
                "-tn", "xterm-256color",
                "-geometry", "140x40",
                "-e", "bash", "-c", bash_cmd]

    env = os.environ.copy()
    # Make sure DISPLAY is forwarded so Gazebo GUI can open
    if "DISPLAY" not in env:
        env["DISPLAY"] = ":0"

    return _register(subprocess.Popen(args, env=env))


# ── Curses colour setup ───────────────────────────────────────────────────────
# Pair index → usage
#  1  normal text
#  2  selected item  (black on cyan)
#  3  header / title (yellow)
#  4  success        (green)
#  5  error          (red)
#  6  dim / hint     (via A_DIM)
#  7  accent / cmd   (cyan)

def _setup_colors() -> None:
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_WHITE,  -1)
    curses.init_pair(2, curses.COLOR_BLACK,  curses.COLOR_CYAN)
    curses.init_pair(3, curses.COLOR_YELLOW, -1)
    curses.init_pair(4, curses.COLOR_GREEN,  -1)
    curses.init_pair(5, curses.COLOR_RED,    -1)
    curses.init_pair(6, curses.COLOR_WHITE,  -1)
    curses.init_pair(7, curses.COLOR_CYAN,   -1)


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


# ── Reset overlay (available in scenario screen via R) ────────────────────────
_RESET_OPTS = [
    ("Soft Reset",       f"bash {RESET_SH} soft"),
    ("Hard Reset",       f"bash {RESET_SH} hard"),
    ("Kill Gazebo/PX4",  'pkill -9 -f "gz"; pkill -9 -f "px4"; pkill -9 -f "gzserver"'),
]

def _reset_overlay(stdscr) -> None:
    mh, mw = 9, 46
    sh, sw = stdscr.getmaxyx()
    my = max(0, (sh - mh) // 2)
    mx = max(0, (sw - mw) // 2)

    sel = 0
    while True:
        ov = curses.newwin(mh, mw, my, mx)
        ov.box()
        ov.addstr(0, 3, "  RESET MENU  ", curses.color_pair(5) | curses.A_BOLD)

        for i, (label, _) in enumerate(_RESET_OPTS):
            row = f" [{i + 1}] {label}"
            try:
                if i == sel:
                    ov.addstr(2 + i, 2, f"{row:<{mw - 4}}", curses.color_pair(2) | curses.A_BOLD)
                else:
                    ov.addstr(2 + i, 2, row, curses.color_pair(1))
            except curses.error:
                pass

        try:
            ov.addstr(mh - 2, 2, " [ESC]  Cancel", curses.color_pair(6) | curses.A_DIM)
        except curses.error:
            pass
        ov.refresh()

        key = ov.getch()
        if key == curses.KEY_UP and sel > 0:
            sel -= 1
        elif key == curses.KEY_DOWN and sel < len(_RESET_OPTS) - 1:
            sel += 1
        elif key in (curses.KEY_ENTER, 10, 13):
            label, cmd = _RESET_OPTS[sel]
            curses.endwin()
            print(f"\n{YLW}[RESET]{RST} {label}...")
            subprocess.run(cmd, shell=True, executable="/bin/bash")
            print(f"\n{GRN}[RESET]{RST} Done. Press ENTER to continue...")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                pass
            stdscr.touchwin()
            stdscr.refresh()
            return
        elif key == 27:  # ESC
            stdscr.touchwin()
            stdscr.refresh()
            return


# ── Screen: World selection ───────────────────────────────────────────────────
def _screen_world(stdscr, worlds: list[str], default: str) -> Optional[str]:
    _setup_colors()
    curses.curs_set(0)
    stdscr.keypad(True)

    sel = worlds.index(default) if default in worlds else 0
    top = 0

    while True:
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        _draw_border(stdscr, "Scout Launcher  —  Select Gazebo World")

        try:
            stdscr.addstr(2, 4, "Choose the simulation world:", curses.color_pair(7))
        except curses.error:
            pass

        list_top = 4
        list_bot = h - 4
        vis = max(1, list_bot - list_top)

        # Scroll tracking
        if sel < top:
            top = sel
        elif sel >= top + vis:
            top = sel - vis + 1

        for i in range(min(vis, len(worlds) - top)):
            idx = top + i
            _draw_row(stdscr, list_top + i, w, worlds[idx], idx == sel)

        # Scroll hint
        if len(worlds) > vis:
            pct = int(sel / max(1, len(worlds) - 1) * 100)
            try:
                stdscr.addstr(h - 3, w - 10, f"({pct}%)", curses.color_pair(6) | curses.A_DIM)
            except curses.error:
                pass

        _draw_footer(stdscr, "UP/DOWN  Navigate    ENTER  Confirm    Q  Quit")
        stdscr.refresh()

        key = stdscr.getch()
        if key == curses.KEY_UP and sel > 0:
            sel -= 1
        elif key == curses.KEY_DOWN and sel < len(worlds) - 1:
            sel += 1
        elif key in (curses.KEY_ENTER, 10, 13):
            return worlds[sel]
        elif key in (ord("q"), ord("Q"), 27):
            return None


# ── Screen: Drone model selection ────────────────────────────────────────────
def _screen_model(stdscr, models: list[tuple[str, str]], default: str) -> Optional[str]:
    _setup_colors()
    curses.curs_set(0)
    stdscr.keypad(True)

    names = [m[0] for m in models]
    sel   = names.index(default) if default in names else 0

    while True:
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        _draw_border(stdscr, "Scout Launcher  —  Select Drone Model")

        try:
            stdscr.addstr(2, 4, "Choose drone model to simulate:", curses.color_pair(7))
        except curses.error:
            pass

        list_top = 4
        list_bot = h - 5
        vis = max(1, list_bot - list_top)

        for i in range(min(vis, len(models))):
            make_target, label = models[i]
            row_text = f"{make_target:<32}  {label}"
            _draw_row(stdscr, list_top + i, w, row_text, i == sel)

        # Preview of selected make target
        make_target, label = models[sel]
        try:
            stdscr.addstr(h - 4, 2, "─" * min(w - 4, 70), curses.color_pair(6) | curses.A_DIM)
            stdscr.addstr(h - 3, 4, f"make px4_sitl {make_target}"[:w - 8],
                          curses.color_pair(7) | curses.A_DIM)
        except curses.error:
            pass

        _draw_footer(stdscr, "UP/DOWN  Navigate    ENTER  Confirm    Q  Quit")
        stdscr.refresh()

        key = stdscr.getch()
        if key == curses.KEY_UP and sel > 0:
            sel -= 1
        elif key == curses.KEY_DOWN and sel < len(models) - 1:
            sel += 1
        elif key in (curses.KEY_ENTER, 10, 13):
            return models[sel][0]
        elif key in (ord("q"), ord("Q"), 27):
            return None


# ── Screen: Scenario selection ────────────────────────────────────────────────
def _screen_scenario(
    stdscr, scenarios: list[Scenario]
) -> Optional[tuple["Scenario", bool]]:
    _setup_colors()
    curses.curs_set(0)
    stdscr.keypad(True)

    sel         = 0
    top         = 0
    with_camera = False   # toggled with C

    while True:
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        _draw_border(stdscr, "Scout Launcher  —  Select Mission Scenario")

        # Header row: mission label + camera toggle indicator
        cam_tag = (
            f"  [{GRN if not curses.has_colors() else ''}CAM ON{RST if not curses.has_colors() else ''}]"
            if with_camera else ""
        )
        try:
            stdscr.addstr(2, 4, "Choose a mission to launch:", curses.color_pair(7))
            if with_camera:
                cam_label = "  + camera view"
                stdscr.addstr(2, 4 + len("Choose a mission to launch:"),
                              cam_label, curses.color_pair(4) | curses.A_BOLD)
        except curses.error:
            pass

        list_top = 4
        desc_h   = 4          # lines reserved for description at the bottom
        list_bot = h - desc_h - 3
        vis = max(1, list_bot - list_top)

        if sel < top:
            top = sel
        elif sel >= top + vis:
            top = sel - vis + 1

        for i in range(min(vis, len(scenarios) - top)):
            idx = top + i
            _draw_row(stdscr, list_top + i, w, scenarios[idx].name, idx == sel)

        # Description panel
        s = scenarios[sel]
        sep_y = h - desc_h - 2
        try:
            stdscr.addstr(sep_y,     2, "─" * min(w - 4, 70), curses.color_pair(6) | curses.A_DIM)
            stdscr.addstr(sep_y + 1, 4, s.description[:w - 8],
                          curses.color_pair(6) | curses.A_DIM)
            stdscr.addstr(sep_y + 2, 4, f"$ {s.ros2_command[:w - 8]}",
                          curses.color_pair(7) | curses.A_DIM)
        except curses.error:
            pass

        # Camera toggle status in footer
        cam_status = "ON " if with_camera else "OFF"
        cam_color  = curses.color_pair(4) | curses.A_BOLD if with_camera else curses.color_pair(6) | curses.A_DIM
        footer_y   = h - 2
        footer_base = "UP/DOWN  Navigate    ENTER  Run    C  Camera:"
        try:
            stdscr.addstr(footer_y, 2, footer_base, curses.color_pair(6) | curses.A_DIM)
            stdscr.addstr(footer_y, 2 + len(footer_base) + 1, cam_status, cam_color)
            stdscr.addstr(footer_y, 2 + len(footer_base) + 1 + len(cam_status) + 2,
                          "   R  Reset    Q  Quit", curses.color_pair(6) | curses.A_DIM)
        except curses.error:
            pass

        stdscr.refresh()

        key = stdscr.getch()
        if key == curses.KEY_UP and sel > 0:
            sel -= 1
        elif key == curses.KEY_DOWN and sel < len(scenarios) - 1:
            sel += 1
        elif key in (ord("c"), ord("C")):
            with_camera = not with_camera
        elif key in (curses.KEY_ENTER, 10, 13):
            return scenarios[sel], with_camera
        elif key in (ord("r"), ord("R")):
            _reset_overlay(stdscr)
        elif key in (ord("q"), ord("Q"), 27):
            return None


# ── Screen: Launch mode (single / swarm) ─────────────────────────────────────
_LAUNCH_MODES: list[tuple[str, str]] = [
    ("single", "Single drone  — 1× PX4 SITL, standard workflow"),
    ("swarm",  "Swarm (2 drones) — 2× PX4 SITL, swarm_coordinator"),
]

def _screen_launch_mode(stdscr) -> Optional[str]:
    _setup_colors()
    curses.curs_set(0)
    stdscr.keypad(True)
    sel = 0

    while True:
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        _draw_border(stdscr, "Scout Launcher  —  Launch Mode")

        try:
            stdscr.addstr(2, 4, "Choose launch mode:", curses.color_pair(7))
        except curses.error:
            pass

        for i, (key, label) in enumerate(_LAUNCH_MODES):
            _draw_row(stdscr, 4 + i, w, label, i == sel)

        # Hint for swarm mode
        try:
            stdscr.addstr(h - 4, 2, "─" * min(w - 4, 70), curses.color_pair(6) | curses.A_DIM)
            if sel == 1:  # swarm
                stdscr.addstr(h - 3, 4,
                    "Vyžaduje: tilted_field nebo swarm_field + gz_x500_mono_cam_down",
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


# ── Step 2: Launch sequence ───────────────────────────────────────────────────
def _kill_stale() -> None:
    """Kill any leftover PX4 / Gazebo processes from a previous session."""
    print(f"{YLW}[CLEANUP]{RST} Killing stale PX4 / Gazebo processes...")
    subprocess.run(
        'pkill -9 -f "px4" 2>/dev/null; '
        'pkill -9 -f "gz sim" 2>/dev/null; '
        'pkill -9 -f "gzserver" 2>/dev/null; '
        'pkill -9 -f "ruby.*gz" 2>/dev/null',
        shell=True, executable="/bin/bash",
    )
    time.sleep(2)
    print(f"{GRN}[CLEANUP]{RST} Done.\n")


def _launch_sequence(world: str, model: str) -> None:
    print(f"\n{BOLD}{BLU}{'━' * 46}{RST}")
    print(f"{BOLD}{BLU}  Launch Sequence{RST}")
    print(f"{BOLD}{BLU}{'━' * 46}{RST}\n")

    # Always kill stale instances first — prevents "PX4 already running" error
    _kill_stale()

    T = 5  # total steps

    # 1/5  PX4 + Gazebo
    print(_step(1, T, f"Starting PX4 SITL + Gazebo  (world: {CYN}{world}{RST}, model: {CYN}{model}{RST})..."))
    _open_terminal(
        f"PX4 SITL — {world}",
        f"cd {PX4_DIR} && PX4_GZ_WORLD={world} make px4_sitl {model}",
    )
    print(_dim("gnome-terminal opened"))

    # 2/5  Wait for PX4
    _countdown(2, T, "Waiting for PX4 to initialize", 15)

    # 3/5  DDS bridge
    print(_step(3, T, "Starting MicroXRCE-DDS bridge..."))
    _open_terminal("MicroXRCE-DDS", "MicroXRCEAgent udp4 -p 8888")
    print(_dim("gnome-terminal opened"))

    # 4/5  Wait for DDS
    _countdown(4, T, "Waiting for DDS bridge", 5)

    # 5/5  QGroundControl
    print(_step(5, T, "Starting QGroundControl..."))
    _open_terminal("QGroundControl", QGC_PATH)
    print(_dim("gnome-terminal opened"))

    print(f"\n{GRN}{BOLD}All services launched!{RST}\n")


def _launch_sequence_swarm(world: str, model: str) -> None:
    """Launch 2-drone swarm: instance 0 first, then instance 1 via PX4_GZ_STANDALONE."""
    print(f"\n{BOLD}{BLU}{'━' * 46}{RST}")
    print(f"{BOLD}{BLU}  Swarm Launch Sequence (2 drones){RST}")
    print(f"{BOLD}{BLU}{'━' * 46}{RST}\n")

    _kill_stale()

    T = 7

    # 1/7  Drone 0 — spawns Gazebo + first drone
    print(_step(1, T, f"Starting drone 0 + Gazebo  (world: {CYN}{world}{RST}, model: {CYN}{model}{RST})..."))
    _open_terminal(
        f"PX4 SITL drone_0 — {world}",
        f"cd {PX4_DIR} && PX4_GZ_WORLD={world} make px4_sitl {model}",
    )
    print(_dim("terminal opened"))

    # 2/7  Wait longer — Gazebo + first drone must be fully up before spawning 2nd
    _countdown(2, T, "Waiting for drone_0 + Gazebo to initialize", 20)

    # 3/7  Drone 1 — connects to already-running Gazebo, spawns on pad_1
    # pad_1 is at Gazebo(3, 0) = NED(0, 3)  [east of pad_0, away from swarm_center]
    # Must cd into build/px4_sitl_default so that etc/init.d-posix/rcS exists
    # KEY: px4-rc.gzsim uses PX4_SIM_MODEL (not PX4_GZ_MODEL) to decide whether to spawn.
    # MODEL_NAME = PX4_SIM_MODEL stripped of "gz_" prefix → "x500_mono_cam_down"
    # MODEL_NAME_INSTANCE = MODEL_NAME + "_" + px4_instance → "x500_mono_cam_down_1"
    print(_step(3, T, f"Starting drone 1  (PX4_GZ_STANDALONE, model: {model}, pose: 3,0,0)..."))
    px4_build = f"{PX4_DIR}/build/px4_sitl_default"
    _open_terminal(
        "PX4 SITL drone_1",
        (
            f"cd {px4_build} && "
            f"PX4_GZ_STANDALONE=1 "
            f"PX4_GZ_WORLD={world} "
            f"PX4_SIM_MODEL={model} "
            f"PX4_GZ_MODEL_POSE='3,0,0,0,0,0' "
            f"./bin/px4 -i 1 -s etc/init.d-posix/rcS"
        ),
    )
    print(_dim("terminal opened"))

    # 4/7  Wait for drone 1
    _countdown(4, T, "Waiting for drone_1 to initialize", 10)

    # 5/7  DDS bridge — one agent handles both via different UXRCE_DDS_KEY
    print(_step(5, T, "Starting MicroXRCE-DDS bridge (shared, port 8888)..."))
    _open_terminal("MicroXRCE-DDS", "MicroXRCEAgent udp4 -p 8888")
    print(_dim("terminal opened"))

    # 6/7  Wait for DDS
    _countdown(6, T, "Waiting for DDS bridge", 5)

    # 7/7  QGroundControl
    print(_step(7, T, "Starting QGroundControl..."))
    _open_terminal("QGroundControl", QGC_PATH)
    print(_dim("terminal opened"))

    print(f"\n{GRN}{BOLD}Swarm launched!{RST}")
    print(f"{GRY}  drone_0 topics:  /fmu/out/...            (MAVLink sysid=1, GCS port 18570){RST}")
    print(f"{GRY}  drone_1 topics:  /px4_1/fmu/out/...      (MAVLink sysid=2, GCS port 18571){RST}")
    print(f"{GRY}  Landing pads:   pad_0 NED(0,0)  pad_1 NED(0,3) [Gz x=0 and x=3]{RST}")
    print(f"{YLW}  QGC multi-vehicle: both drones broadcast to UDP 14550.{RST}")
    print(f"{YLW}  Use the vehicle selector (top toolbar) to switch between them.{RST}\n")


def _countdown(n: int, t: int, label: str, secs: int) -> None:
    for i in range(secs, 0, -1):
        sys.stdout.write(f"\r{_step(n, t, f'{label} ({i}s)...')}   ")
        sys.stdout.flush()
        time.sleep(1)
    sys.stdout.write(f"\r{_step(n, t, label + ' — done')}            \n")
    sys.stdout.flush()


# ── Gazebo model name from PX4 make target ────────────────────────────────────
def _gz_model_name(make_target: str) -> str:
    """Convert PX4 make target to Gazebo instance-0 model name.

    e.g. 'gz_x500_mono_cam' → 'x500_mono_cam_0'
         'gz_x500_mono_cam_lidar' → 'x500_mono_cam_lidar_0'
    """
    name = make_target.removeprefix("gz_")
    return f"{name}_0"


# ── Step 4: Build + Run ───────────────────────────────────────────────────────
def _build_and_run(
    scenario: Scenario,
    with_camera: bool = False,
    world: str = DEFAULT_WORLD,
    model: str = DEFAULT_MODEL,
) -> None:
    print(f"\n{BOLD}{BLU}{'━' * 46}{RST}")
    print(f"{BOLD}{BLU}  {scenario.name}{RST}")
    print(f"{BOLD}{BLU}{'━' * 46}{RST}\n")

    # BUILD (blocking, live output)
    print(f"{CYN}[BUILD]{RST} colcon build --packages-select scout_control\n")
    result = subprocess.run(
        f"cd {WS_DIR} && colcon build --packages-select scout_control",
        shell=True,
        executable="/bin/bash",
        cwd=WS_DIR,
    )

    if result.returncode != 0:
        print(f"\n{_err(f'Build FAILED  (rc={result.returncode})')}")
        print("Fix the errors above, then re-run.\n")
        sys.exit(1)

    print(f"\n{GRN}[BUILD]{RST} success {GRN}✓{RST}")

    # Template substitution: {world}, {model} (make target), {gz_model} (Gazebo instance name)
    gz_model = _gz_model_name(model)
    cmd = scenario.ros2_command.format(world=world, model=model, gz_model=gz_model)

    # SOURCE + RUN in new terminal
    print(f"{CYN}[SOURCE]{RST} install/setup.bash")
    print(f"{CYN}[RUN]{RST}    {cmd}\n")

    ros2_full = (
        f"source {ROS2_SETUP} && "
        f"source {WS_SETUP} && "
        f"{cmd}"
    )
    _open_terminal(scenario.name, ros2_full)

    # Optional camera visualisation (toggled with C in the scenario screen)
    if with_camera:
        cam_cmd = (
            f"source {ROS2_SETUP} && "
            f"source {WS_SETUP} && "
            f"ros2 run rqt_image_view rqt_image_view /camera/image_raw"
        )
        _open_terminal("Camera — /camera/image_raw", cam_cmd)
        print(_dim("Extra terminal opened: rqt_image_view /camera/image_raw"))

    # Extra terminals declared in scenario yaml (e.g. curses UI nodes that need a real TTY)
    for extra_cmd_template in scenario.extra_terminal_commands:
        extra_cmd = extra_cmd_template.format(world=world, model=model, gz_model=gz_model)
        extra_full = (
            f"source {ROS2_SETUP} && "
            f"source {WS_SETUP} && "
            f"{extra_cmd}"
        )
        # derive a readable title from the command
        title = extra_cmd.split("ros2 run ")[-1].split(" ")[1] if "ros2 run" in extra_cmd else extra_cmd[:40]
        _open_terminal(title, extra_full)
        print(_dim(f"Extra terminal: {extra_cmd}"))

    print(f"{_ok(f'{scenario.name!r} launched in new terminal.')}\n")


# ── Preflight checks ─────────────────────────────────────────────────────────
def _check_python_symlink() -> None:
    """Ensure /usr/bin/python exists — required by PX4 build scripts.

    Ubuntu 24.04 ships only python3; the symlink is created by the
    'python-is-python3' package.  Without it `make px4_sitl` fails with
    '/usr/bin/python: not found'.
    """
    if Path("/usr/bin/python").exists():
        return

    print(f"{YLW}[PREFLIGHT]{RST} /usr/bin/python not found — PX4 build will fail.")
    print(f"{YLW}[PREFLIGHT]{RST} Installing python-is-python3 (requires sudo)...")
    result = subprocess.run(
        ["sudo", "apt-get", "install", "-y", "python-is-python3"],
        check=False,
    )
    if result.returncode == 0:
        print(f"{GRN}[PREFLIGHT]{RST} python-is-python3 installed {GRN}✓{RST}\n")
    else:
        print(f"{RED}[PREFLIGHT]{RST} Installation failed (rc={result.returncode}).")
        print(f"            Run manually:  sudo apt install python-is-python3\n")


# ── Signal handler ────────────────────────────────────────────────────────────
def _on_sigint(sig, frame) -> None:
    # Make sure curses is cleaned up
    try:
        curses.endwin()
    except Exception:
        pass
    print(f"\n{YLW}Shutting down...{RST}")
    _kill_all()
    sys.exit(0)


# ── Header banner ─────────────────────────────────────────────────────────────
_BANNER = f"""\
{CYN}{BOLD}
  ╔══════════════════════════════════════╗
  ║       Scout Launcher                 ║
  ║   ROS2 · PX4 · Gazebo Harmonic      ║
  ╚══════════════════════════════════════╝
{RST}"""


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    locale.setlocale(locale.LC_ALL, "")
    signal.signal(signal.SIGINT, _on_sigint)

    cfg        = _load_cfg()
    last_world = cfg.get("world", DEFAULT_WORLD)
    last_model = cfg.get("model", DEFAULT_MODEL)

    os.system("clear")
    print(_BANNER)

    # ── Preflight ─────────────────────────────────────────────────────────────
    _check_python_symlink()

    # ── Step 1: World selection ───────────────────────────────────────────────
    worlds = _scan_worlds()
    world = curses.wrapper(_screen_world, worlds, last_world)
    if world is None:
        print("Aborted.")
        sys.exit(0)

    cfg["world"] = world
    _save_cfg(cfg)
    print(f"{_ok(f'World selected: {CYN}{world}{RST}')}\n")

    # ── Step 1b: Drone model selection ───────────────────────────────────────
    model = curses.wrapper(_screen_model, DRONE_MODELS, last_model)
    if model is None:
        print("Aborted.")
        sys.exit(0)

    cfg["model"] = model
    _save_cfg(cfg)
    print(f"{_ok(f'Model selected: {CYN}{model}{RST}')}\n")

    # ── Step 1c: Launch mode selection ───────────────────────────────────────
    launch_mode = curses.wrapper(_screen_launch_mode)
    if launch_mode is None:
        print("Aborted.")
        sys.exit(0)
    print(f"{_ok(f'Mode: {CYN}{launch_mode}{RST}')}\n")

    # ── Step 2: Launch sequence ───────────────────────────────────────────────
    if launch_mode == "swarm":
        _launch_sequence_swarm(world, model)
    else:
        _launch_sequence(world, model)

    # ── Steps 3+4: Scenario loop — runs until user presses Q ────────────────
    scenarios = _scan_scenarios()
    if not scenarios:
        print(f"{YLW}No scenarios found in:{RST}  {SCENARIOS_DIR}/")
        print("Create .yaml files with fields: name, description, ros2_command\n")
        sys.exit(0)

    last_scenario_idx = 0
    while True:
        print(f"\n{GRY}{'─' * 46}{RST}")
        print(f"{GRY}All services running. Select next scenario (Q to quit).{RST}\n")

        # Re-scan on each iteration so newly added .yaml files appear
        scenarios = _scan_scenarios()

        result = curses.wrapper(_screen_scenario, scenarios)
        if result is None:
            print(f"\n{YLW}Exiting launcher.{RST}")
            _kill_all()
            break

        scenario, with_camera = result
        cam_note = f"  {GRN}+ camera view{RST}" if with_camera else ""
        print(f"{_ok(f'Scenario: {CYN}{scenario.name}{RST}')}{cam_note}\n")
        _build_and_run(scenario, with_camera, world=world, model=model)

        # Brief pause so the user can read the output before curses redraws
        print(f"\n{GRY}Press ENTER to select another scenario, or Q + ENTER to quit...{RST}")
        try:
            ans = input()
        except (EOFError, KeyboardInterrupt):
            break
        if ans.strip().lower() == "q":
            print(f"{YLW}Exiting launcher.{RST}")
            _kill_all()
            break


if __name__ == "__main__":
    main()
