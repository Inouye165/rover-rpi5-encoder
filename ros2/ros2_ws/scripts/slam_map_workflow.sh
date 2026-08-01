#!/usr/bin/env bash
# ==============================================================================
# slam_map_workflow.sh - Passive SLAM Map Save & Verification Utility
# ==============================================================================
# Supports safe map save, list, verify, and load workflows with strict safety
# checks (disarmed, autonomy disabled, maintenance inactive) and name validation.
# ==============================================================================

set -euo pipefail

MAP_DIR="/ros2_ws/maps"
COCKPIT_URL="http://127.0.0.1:3000"
TIMEOUT_SEC=10

# Create map directory if missing
mkdir -p "${MAP_DIR}"

log_info() {
    echo "[SLAM Workflow INFO] $1"
}

log_error() {
    echo "[SLAM Workflow ERROR] $1" >&2
}

# 1. Safety Checks (Disarmed, Autonomy Disabled, Maintenance Inactive)
check_safety_state() {
    log_info "Performing safety state verification..."

    # Check Cockpit Drive Status
    local drive_res
    drive_res=$(curl -s --max-time 3 "${COCKPIT_URL}/api/drive/status" || echo '{"ok":false}')
    local is_armed
    is_armed=$(echo "${drive_res}" | grep -o '"armed":[^,}]*' | cut -d: -f2 | tr -d ' ' || echo "true")
    if [ "${is_armed}" != "false" ]; then
        log_error "Safety Check Failed: Rover is ARMED (${is_armed}). Aborting operation!"
        exit 1
    fi

    # Check Autonomy Status
    local aut_res
    aut_res=$(curl -s --max-time 3 "${COCKPIT_URL}/api/autonomy/status" || echo '{"ok":false}')
    local aut_state
    aut_state=$(echo "${aut_res}" | grep -o '"state":"[^"]*' | cut -d'"' -f4 || echo "ENABLED")
    if [ "${aut_state}" != "DISABLED" ]; then
        log_error "Safety Check Failed: Autonomy state is ${aut_state} (expected DISABLED). Aborting operation!"
        exit 1
    fi

    # Check Maintenance Status
    local maint_res
    maint_res=$(curl -s --max-time 3 "${COCKPIT_URL}/api/maintenance/status" || echo '{"ok":false}')
    local maint_active
    maint_active=$(echo "${maint_res}" | grep -o '"active":[^,}]*' | cut -d: -f2 | tr -d ' ' || echo "true")
    if [ "${maint_active}" != "false" ]; then
        log_error "Safety Check Failed: Maintenance mode is ACTIVE. Aborting operation!"
        exit 1
    fi

    log_info "Safety state PASS: Disarmed=true, Autonomy=DISABLED, Maintenance=inactive."
}

# 2. Strict Map Name Validation
validate_map_name() {
    local name="$1"
    if [[ ! "${name}" =~ ^[a-zA-Z0-9_-]+$ ]]; then
        log_error "Invalid map name '${name}'. Map name must contain only alphanumeric characters, underscores, and hyphens."
        exit 1
    fi

    # Protect core production map names from accidental overwrites
    case "${name}" in
        house_map|home|final|production)
            log_error "Protected map name '${name}' cannot be used for disposable/test operations."
            exit 1
            ;;
    esac
}

# 3. Status Command
cmd_status() {
    check_safety_state
    
    # Check SLAM Toolbox lifecycle state
    local state
    state=$(timeout "${TIMEOUT_SEC}" ros2 lifecycle get /slam_toolbox 2>/dev/null || echo "not running")
    log_info "/slam_toolbox state: ${state}"

    # Check /map topic
    local map_info
    map_info=$(timeout "${TIMEOUT_SEC}" ros2 topic info /map 2>/dev/null || echo "no map topic")
    log_info "/map topic info: ${map_info}"
}

# 4. Save Command
cmd_save() {
    local map_name="${1:-}"
    if [ -z "${map_name}" ]; then
        log_error "Usage: $0 save <map_name>"
        exit 1
    fi

    validate_map_name "${map_name}"
    check_safety_state

    # Verify SLAM Toolbox is active
    local state
    state=$(timeout "${TIMEOUT_SEC}" ros2 lifecycle get /slam_toolbox 2>/dev/null || echo "not active")
    if [[ "${state}" != *"active"* ]]; then
        log_error "/slam_toolbox is not ACTIVE (current state: ${state}). Aborting map save!"
        exit 1
    fi

    # Verify /map is publishing
    local pubs
    pubs=$(ros2 topic info /map 2>/dev/null | grep "Publisher count:" | awk '{print $3}' || echo "0")
    if [ "${pubs}" -lt 1 ]; then
        log_error "/map topic has 0 publishers. Aborting map save!"
        exit 1
    fi

    local target_prefix="${MAP_DIR}/${map_name}"
    log_info "Saving Occupancy Map to ${target_prefix}.yaml / .pgm..."

    # Call /slam_toolbox/save_map (SaveMap: std_msgs/String name)
    local save_res
    save_res=$(timeout "${TIMEOUT_SEC}" ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '${target_prefix}'}}" 2>&1 || echo "ERROR")
    if [[ "${save_res}" != *"result=0"* ]]; then
        log_error "Failed to save occupancy map: ${save_res}"
        exit 1
    fi

    log_info "Serializing SLAM PoseGraph to ${target_prefix}.posegraph / .data..."
    # Call /slam_toolbox/serialize_map (SerializePoseGraph: string filename)
    local ser_res
    ser_res=$(timeout "${TIMEOUT_SEC}" ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '${target_prefix}'}" 2>&1 || echo "ERROR")
    if [[ "${ser_res}" != *"result=0"* ]]; then
        log_error "Failed to serialize pose graph: ${ser_res}"
        exit 1
    fi

    log_info "Verifying saved map artifacts..."
    cmd_verify "${map_name}"
}

# 5. Verify Command
cmd_verify() {
    local map_name="${1:-}"
    if [ -z "${map_name}" ]; then
        log_error "Usage: $0 verify <map_name>"
        exit 1
    fi

    validate_map_name "${map_name}"
    local target_prefix="${MAP_DIR}/${map_name}"
    local errors=0

    for ext in yaml pgm posegraph data; do
        local file="${target_prefix}.${ext}"
        if [ ! -f "${file}" ]; then
            log_error "Missing artifact: ${file}"
            errors=$((errors + 1))
        elif [ ! -s "${file}" ]; then
            log_error "Empty artifact: ${file}"
            errors=$((errors + 1))
        else
            local size
            size=$(wc -c < "${file}")
            log_info "Artifact VERIFIED: $(basename "${file}") (${size} bytes)"
        fi
    done

    if [ "${errors}" -gt 0 ]; then
        log_error "Map verification FAILED with ${errors} error(s)."
        exit 1
    fi

    log_info "YAML Metadata Preview (${map_name}.yaml):"
    cat "${target_prefix}.yaml" | head -n 10
}

# 6. List Command
cmd_list() {
    log_info "Available maps in ${MAP_DIR}:"
    if [ -d "${MAP_DIR}" ]; then
        ls -la "${MAP_DIR}"/*.yaml 2>/dev/null || log_info "No YAML maps found in ${MAP_DIR}."
    fi
}

# 7. Main Dispatcher
case "${1:-}" in
    status)
        cmd_status
        ;;
    save)
        cmd_save "${2:-}"
        ;;
    verify)
        cmd_verify "${2:-}"
        ;;
    list)
        cmd_list
        ;;
    *)
        echo "Usage: $0 {status|save <map_name>|verify <map_name>|list}"
        exit 1
        ;;
esac
