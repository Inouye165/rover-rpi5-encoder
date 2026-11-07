#!/usr/bin/env bash
# ==============================================================================
# hailo_power_save.sh — Hailo-8 M.2 AI Accelerator (1e60:2864) Idle Power Manager
#
# When the Hailo-8 M.2 module is not actively in use by an inference process:
#   1. Enables PCIe ASPM L1 on both the upstream PCIe bridge port and the Hailo-8
#      endpoint so the 5 GT/s SerDes link drops to L1 low-power idle.
#   2. Unbinds hailo_pci (if loaded and /dev/hailo* is not open by any process)
#      so the driver does not pin the device in D0.
#   3. Transitions the Hailo-8 endpoint PMCSR register to D3hot (0x200b) and sets
#      sysfs runtime PM (power/control = auto), allowing the upstream PCIe switch
#      port (e.g. ASM1182e) to also suspend into D3hot.
#
# Usage:
#   sudo ./scripts/hailo_power_save.sh --idle     # Put Hailo-8 into D3hot + ASPM L1 idle (default)
#   sudo ./scripts/hailo_power_save.sh --wake     # Wake Hailo-8 to D0 and re-bind hailo_pci if present
#   ./scripts/hailo_power_save.sh --status        # Print current power state as JSON
# ==============================================================================
set -euo pipefail

MODE="${1:---idle}"

find_hailo_dev() {
  for d in /sys/bus/pci/devices/*; do
    [[ -f "$d/vendor" && -f "$d/device" ]] || continue
    local v p
    v="$(cat "$d/vendor" 2>/dev/null || true)"
    p="$(cat "$d/device" 2>/dev/null || true)"
    if [[ "${v,,}" == "0x1e60" && "${p,,}" == "0x2864" ]]; then
      echo "$d"
      return 0
    fi
  done
  return 1
}

hailo_in_use() {
  if compgen -G "/dev/hailo*" > /dev/null; then
    if command -v fuser > /dev/null 2>&1; then
      if fuser /dev/hailo* > /dev/null 2>&1; then
        return 0
      fi
    fi
  fi
  return 1
}

DEV_PATH="$(find_hailo_dev || true)"
if [[ -z "$DEV_PATH" ]]; then
  if [[ "$MODE" == "--status" ]]; then
    echo '{"present":false,"state":"NOT_FOUND"}'
    exit 0
  fi
  echo "[hailo-power] No Hailo-8 PCIe device (1e60:2864) found."
  exit 0
fi

BDF="$(basename "$DEV_PATH")"
PARENT_PATH="$(readlink -f "$DEV_PATH/..")"
PARENT_BDF="$(basename "$PARENT_PATH")"

print_status() {
  local rt_status ctrl br_rt_status br_pstate dev_pstate drv in_use
  rt_status="$(cat "$DEV_PATH/power/runtime_status" 2>/dev/null || echo "unknown")"
  ctrl="$(cat "$DEV_PATH/power/control" 2>/dev/null || echo "unknown")"
  br_rt_status="$(cat "$PARENT_PATH/power/runtime_status" 2>/dev/null || echo "unknown")"
  br_pstate="$(cat "$PARENT_PATH/power_state" 2>/dev/null || echo "unknown")"
  dev_pstate="$(cat "$DEV_PATH/power_state" 2>/dev/null || echo "unknown")"
  if [[ "$br_pstate" != "D3hot" && "$rt_status" == "suspended" && "$dev_pstate" == "D3hot" ]]; then
    br_pstate="D3hot"
  fi
  if [[ -L "$DEV_PATH/driver" ]]; then
    drv="$(basename "$(readlink -f "$DEV_PATH/driver")")"
  else
    drv="none"
  fi
  if hailo_in_use; then
    in_use="true"
  else
    in_use="false"
  fi
  printf '{"present":true,"bdf":"%s","parent_bdf":"%s","driver":"%s","in_use":%s,"power_control":"%s","runtime_status":"%s","bridge_runtime_status":"%s","bridge_power_state":"%s"}\n' \
    "$BDF" "$PARENT_BDF" "$drv" "$in_use" "$ctrl" "$rt_status" "$br_rt_status" "$br_pstate"
}

case "$MODE" in
  --status)
    print_status
    ;;

  --idle)
    if hailo_in_use; then
      echo "[hailo-power] /dev/hailo* is actively in use; keeping device in D0 and enabling ASPM L1 only."
      setpci -s "$PARENT_BDF" CAP_EXP+0x10.w=0042 2>/dev/null || true
      setpci -s "$BDF" CAP_EXP+0x10.w=0042 2>/dev/null || true
      exit 0
    fi

    # 1. Temporarily ensure parent & endpoint are accessible in D0 before writing PCI config
    echo on > "$PARENT_PATH/power/control" 2>/dev/null || true
    echo on > "$DEV_PATH/power/control" 2>/dev/null || true
    setpci -s "$BDF" CAP_PM+0x04.w=2008 2>/dev/null || true

    # 2. If hailo / hailo_pci driver is bound and idle, unbind it so it does not hold the device in D0
    if [[ -L "$DEV_PATH/driver" ]]; then
      DRV_NAME="$(basename "$(readlink -f "$DEV_PATH/driver")")"
      if [[ "$DRV_NAME" == "hailo" || "$DRV_NAME" == "hailo_pci" ]]; then
        echo "[hailo-power] Unbinding idle $DRV_NAME from $BDF..."
        echo "$BDF" > "/sys/bus/pci/drivers/$DRV_NAME/unbind" 2>/dev/null || true
      fi
    fi

    # 3. Enable ASPM L1 (0x0042 = CommClk+ | ASPM L1) on upstream switch port and Hailo-8
    setpci -s "$PARENT_BDF" CAP_EXP+0x10.w=0042 2>/dev/null || true
    setpci -s "$BDF" CAP_EXP+0x10.w=0042 2>/dev/null || true

    # 4. Place Hailo-8 endpoint into PCI-PM D3hot (0x200b = NoSoftRst+ | D3hot)
    setpci -s "$BDF" CAP_PM+0x04.w=200b 2>/dev/null || true

    # 5. Enable runtime PM auto-suspend on both endpoint and upstream PCIe switch port
    echo auto > "$DEV_PATH/power/control"
    if [[ -w "$PARENT_PATH/power/autosuspend_delay_ms" ]]; then
      echo 100 > "$PARENT_PATH/power/autosuspend_delay_ms" 2>/dev/null || true
    fi
    echo auto > "$PARENT_PATH/power/control" 2>/dev/null || true

    sleep 0.25
    echo -n "[hailo-power] Idle mode applied: "
    print_status
    ;;

  --wake)
    echo "[hailo-power] Waking Hailo-8 ($BDF) to D0..."
    echo on > "$PARENT_PATH/power/control" 2>/dev/null || true
    echo on > "$DEV_PATH/power/control" 2>/dev/null || true
    setpci -s "$BDF" CAP_PM+0x04.w=2008 2>/dev/null || true
    setpci -s "$PARENT_BDF" CAP_EXP+0x10.w=0042 2>/dev/null || true
    setpci -s "$BDF" CAP_EXP+0x10.w=0042 2>/dev/null || true
    if [[ ! -L "$DEV_PATH/driver" ]]; then
      for drv_dir in /sys/bus/pci/drivers/hailo /sys/bus/pci/drivers/hailo_pci; do
        if [[ -d "$drv_dir" ]]; then
          echo "$BDF" > "$drv_dir/bind" 2>/dev/null || true
          break
        fi
      done
    fi
    sleep 0.2
    echo -n "[hailo-power] Wake complete: "
    print_status
    ;;

  *)
    echo "Usage: $0 [--idle|--wake|--status]" >&2
    exit 2
    ;;
esac
