#!/usr/bin/env bash
# =============================================================================
# reset.sh — Drone emergency reset
#
# Použití:
#   ./reset.sh          — soft reset (Gazebo reset + restart ROS2 nodu)
#   ./reset.sh hard     — hard reset (zabij vše, restartuj v nových terminálech)
# =============================================================================

WS=/home/tj/_Data/_Projekty/TJlabs/scout_ws
GZ_WORLD=${PX4_GZ_WORLD:-agricultural_field}
PX4_DIR=/home/tj/PX4-Autopilot

RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
NC='\033[0m'

log()  { echo -e "${GRN}[RESET]${NC} $*"; }
warn() { echo -e "${YLW}[RESET]${NC} $*"; }
err()  { echo -e "${RED}[RESET]${NC} $*"; }

# ─────────────────────────────────────────────────────────────────────────────
kill_ros2_nodes() {
    log "Ukončuji scout_control ROS2 nody..."
    pkill -f "offboard_control" 2>/dev/null
    pkill -f "perimeter_flight" 2>/dev/null
    pkill -f "grid_generator"   2>/dev/null
    pkill -f "field_commander"  2>/dev/null
    pkill -f "camera_bridge"    2>/dev/null
    pkill -f "image_bridge"     2>/dev/null
    sleep 1
}

# ─────────────────────────────────────────────────────────────────────────────
soft_reset() {
    log "=== SOFT RESET ==="
    log "World: $GZ_WORLD"

    # 1) Zastav ROS2 nody (dron přestane dostávat setpointy)
    kill_ros2_nodes

    # 2) Resetuj Gazebo world — vrátí dron na spawn pozici, resetuje PX4 state
    log "Volám gz world reset..."
    gz service \
        -s "/world/${GZ_WORLD}/control" \
        --reqtype gz.msgs.WorldControl \
        --reptype gz.msgs.Boolean \
        --timeout 3000 \
        --req 'reset: {all: true}' 2>/dev/null

    if [ $? -eq 0 ]; then
        log "Gazebo reset OK — dron je zpět na startovní pozici"
    else
        err "Gazebo reset selhal (Gazebo neběží?) — zkus: ./reset.sh hard"
        exit 1
    fi

    # 3) Počkej na PX4 boot po resetu
    log "Čekám 4s na PX4 re-init..."
    sleep 4

    # 4) Restartuj ROS2 node v novém terminálu
    log "Spouštím ROS2 node..."
    xterm -title "scout_control" -e bash -c "
        source /opt/ros/jazzy/setup.bash
        source ${WS}/install/setup.bash
        echo '--- scout_control node ---'
        ros2 launch scout_control camera_bridge.launch.py world:=${GZ_WORLD}
        read -p 'Press Enter to close'
    " &

    log "=== Soft reset hotovo ==="
    log "Tip: Pro perimeter misi spusť místo toho:"
    log "  ros2 run scout_control perimeter_flight"
}

# ─────────────────────────────────────────────────────────────────────────────
hard_reset() {
    log "=== HARD RESET — zabíjím vše a restartuji ==="
    warn "QGroundControl se odpojí a znovu připojí automaticky."

    # 1) Zastav ROS2 nody
    kill_ros2_nodes

    # 2) Zastav MicroXRCE
    log "Ukončuji MicroXRCEAgent..."
    pkill -f "MicroXRCEAgent" 2>/dev/null
    sleep 1

    # 3) Zastav Gazebo + PX4 (gz-sim plugin)
    log "Ukončuji Gazebo + PX4..."
    pkill -9 -f "gz sim"     2>/dev/null
    pkill -9 -f "gzserver"   2>/dev/null
    pkill -9 -f "px4"        2>/dev/null
    sleep 3

    log "Procesy zastaveny. Spouštím vše znovu..."

    # 4) PX4 SITL + Gazebo (nový terminál)
    xterm -title "PX4 SITL" -geometry 120x30 -e bash -c "
        cd ${PX4_DIR}
        echo '--- PX4 SITL + Gazebo ---'
        PX4_GZ_WORLD=${GZ_WORLD} make px4_sitl gz_x500_mono_cam
        read -p 'Press Enter to close'
    " &
    PX4_XTERM=$!

    log "Čekám 8s na PX4 + Gazebo startup..."
    sleep 8

    # 5) MicroXRCE bridge (nový terminál)
    xterm -title "MicroXRCE" -geometry 80x20 -e bash -c "
        echo '--- MicroXRCEAgent ---'
        MicroXRCEAgent udp4 -p 8888
        read -p 'Press Enter to close'
    " &

    log "Čekám 3s na MicroXRCE..."
    sleep 3

    # 6) ROS2 node (nový terminál)
    xterm -title "scout_control" -geometry 120x30 -e bash -c "
        source /opt/ros/jazzy/setup.bash
        source ${WS}/install/setup.bash
        echo '--- scout_control launch ---'
        ros2 launch scout_control camera_bridge.launch.py world:=${GZ_WORLD}
        read -p 'Press Enter to close'
    " &

    log "=== Hard reset hotovo ==="
    log "3 nové terminály otevřeny: PX4, MicroXRCE, scout_control"
    warn "QGroundControl se připojí automaticky (UDP 14550)"
}

# ─────────────────────────────────────────────────────────────────────────────
case "${1:-soft}" in
    hard|--hard|-h)
        hard_reset
        ;;
    soft|--soft|-s|"")
        soft_reset
        ;;
    *)
        echo "Použití: $0 [soft|hard]"
        echo "  soft  — Gazebo world reset + restart ROS2 nodu (rychlý)"
        echo "  hard  — kill vše + restart v nových terminálech (spolehlivý)"
        exit 1
        ;;
esac
