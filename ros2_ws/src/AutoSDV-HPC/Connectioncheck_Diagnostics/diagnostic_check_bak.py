#!/usr/bin/env python3
"""
AutoSDV Connectivity Diagnostic
================================
Pre-flight checklist for SDV experiments.
Checks L0 (HPC environment), L1 (network), L3 (DDS data flow).

Usage:
    source /opt/ros/humble/setup.bash
    python3 diagnostic_check.py
"""

import os
import re
import shutil
import subprocess
import sys

# ── Configuration ──────────────────────────────────────────────────────────

ZONES = {
    "Zone1 - Steering": {
        "ip": "10.0.0.2",
        "cameras": {
            "fisheye": "/image_02_raw",
            "webcam":  "/image_01_raw",
        },
        "ecu_status": "/topic_status_signal",
    },
    "Zone2 - Front_Left": {
        "ip": "10.0.0.3",
        "cameras": {
            "fisheye": "/image_03_raw",
        },
        "ecu_status": "/topic_status_signal",
    },
    "Zone3 - Front_Right": {
        "ip": "10.0.0.4",
        "cameras": {
            "fisheye": "/image_04_raw",
        },
        "ecu_status": "/topic_status_signal",
    },
    "Zone4 - Rear": {
        "ip": "10.0.0.5",
        "cameras": {
            "fisheye": "/image_05_raw",
        },
        "ecu_status": "/topic_status_signal",
    },
}

TIMEOUT = 3  # L3 topic echo timeout (seconds)

# ── Exit codes ─────────────────────────────────────────────────────────────

EXIT_ALL_OK   = 0
EXIT_PARTIAL  = 1
EXIT_CRITICAL = 2
EXIT_ENV_ERR  = 3

# ── ANSI colors ────────────────────────────────────────────────────────────

GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

# ── Result tracking ────────────────────────────────────────────────────────

results = []  # list of (zone_name, check_name, status, detail)
              # status: True=OK, False=FAIL, None=SKIP


def record(zone, check, status, detail=""):
    results.append((zone, check, status, detail))


# ── Output helpers ─────────────────────────────────────────────────────────

def tag(status):
    if status is True:
        return f"{GREEN}[OK]{RESET}  "
    elif status is False:
        return f"{RED}[FAIL]{RESET}"
    else:
        return f"{YELLOW}[SKIP]{RESET}"


def print_check(label, status, detail=""):
    dots = "." * max(1, 35 - len(label))
    detail_str = f"  {detail}" if detail else ""
    print(f"  +-- {label} {dots} {tag(status)}{detail_str}")


def print_header(title):
    print(f"\n{BOLD}[{title}]{RESET}")


# ── L0: HPC Environment ───────────────────────────────────────────────────

def check_l0():
    print_header("L0  HPC Environment")
    all_ok = True

    # ROS2 installation
    ros2_path = shutil.which("ros2")
    ok = ros2_path is not None
    record("L0", "ROS2 environment", ok)
    print_check("ROS2 environment", ok)
    if not ok:
        all_ok = False

    # DOMAIN_ID
    domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
    ok = domain_id == "0"
    detail = f"current={domain_id}" if not ok else ""
    record("L0", "ROS_DOMAIN_ID = 0", ok, detail)
    print_check("ROS_DOMAIN_ID = 0", ok, detail)
    if not ok:
        all_ok = False

    return all_ok


# ── L1: Network (ping) ────────────────────────────────────────────────────

def check_ping(ip):
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", "1", ip],
            capture_output=True,
            text=True,
            timeout=3,
        )
        if result.returncode == 0:
            match = re.search(r"time=([\d.]+)\s*ms", result.stdout)
            rtt = match.group(1) if match else "?"
            return True, f"rtt={rtt}ms"
        return False, ""
    except subprocess.TimeoutExpired:
        return False, "timeout"
    except FileNotFoundError:
        return False, "ping not found"


# ── L3: Data flow (DDS topic echo) ────────────────────────────────────────

def check_data_flow(topic, timeout=TIMEOUT, qos_best_effort=False):
    cmd = ["timeout", str(timeout), "ros2", "topic", "echo", topic, "--once"]
    if qos_best_effort:
        cmd += ["--qos-reliability", "best_effort"]
    try:
        result = subprocess.run(cmd, capture_output=True, timeout=timeout + 2)
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        return False
    except FileNotFoundError:
        return False


# ── Zone check (L1 + L3 with short-circuit) ───────────────────────────────

def check_zone(zone_name, zone_config):
    ip = zone_config["ip"]
    cameras = zone_config["cameras"]
    ecu_topic = zone_config["ecu_status"]

    print_header(f"{zone_name}  ZCU: {ip}")

    # L1: ping
    ping_ok, rtt = check_ping(ip)
    record(zone_name, "HPC <-> ZCU", ping_ok, rtt)
    print_check(f"HPC <-> ZCU", ping_ok, rtt)

    if not ping_ok:
        # Short-circuit: skip all L3 checks
        for cam_name in cameras:
            record(zone_name, f"Camera ({cam_name})", None, "ZCU unreachable")
            print_check(f"Camera ({cam_name})", None, "(ZCU unreachable)")
        record(zone_name, "ECU (CAN)", None, "ZCU unreachable")
        print_check("ECU (CAN)", None, "(ZCU unreachable)")
        return

    # L3: cameras (BEST_EFFORT QoS — matches ZCU camera publisher)
    for cam_name, topic in cameras.items():
        ok = check_data_flow(topic, qos_best_effort=True)
        detail = topic if ok else f"{topic} (timeout {TIMEOUT}s)"
        record(zone_name, f"Camera ({cam_name})", ok, detail)
        print_check(f"Camera ({cam_name})", ok, detail)

    # L3: ECU
    ok = check_data_flow(ecu_topic)
    detail = ecu_topic if ok else f"{ecu_topic} (timeout {TIMEOUT}s)"
    record(zone_name, "ECU (CAN)", ok, detail)
    print_check("ECU (CAN)", ok, detail)


# ── Summary ────────────────────────────────────────────────────────────────

def print_summary():
    print(f"\n{'=' * 48}")
    print(f"{BOLD}SUMMARY{RESET}")

    zone_statuses = {}
    for zone, check, status, detail in results:
        if zone == "L0":
            continue
        if zone not in zone_statuses:
            zone_statuses[zone] = []
        zone_statuses[zone].append((check, status, detail))

    for zone, checks in zone_statuses.items():
        failures = [c for c, s, d in checks if s is False]
        skips = [c for c, s, d in checks if s is None]

        # Determine zone status
        if any(c == "HPC <-> ZCU" and s is False for c, s, _ in checks):
            status_str = f"{RED}FAIL{RESET}  - ZCU unreachable"
        elif failures:
            failed_names = ", ".join(f.replace("Camera ", "").replace("(", "").replace(")", "")
                                     for f in failures)
            status_str = f"{YELLOW}WARN{RESET}  - {failed_names} offline"
        elif skips:
            status_str = f"{YELLOW}SKIP{RESET}"
        else:
            status_str = f"{GREEN}OK{RESET}"

        print(f"  {zone:28s} {status_str}")

    # Totals
    total = len([r for r in results if r[0] != "L0"])
    passed = len([r for r in results if r[0] != "L0" and r[2] is True])
    failed = len([r for r in results if r[0] != "L0" and r[2] is False])
    skipped = len([r for r in results if r[0] != "L0" and r[2] is None])

    print(f"\n  Total: {passed}/{total} passed, {skipped} skipped, {failed} failed")
    print("=" * 48)

    return passed, failed, skipped


def compute_exit_code():
    l1_fail = any(c == "HPC <-> ZCU" and s is False for zone, c, s, d in results)
    l3_fail = any(zone != "L0" and c != "HPC <-> ZCU" and s is False for zone, c, s, d in results)

    if l1_fail:
        return EXIT_CRITICAL
    elif l3_fail:
        return EXIT_PARTIAL
    else:
        return EXIT_ALL_OK


# ── Failure diagnosis hint ─────────────────────────────────────────────────

DIAGNOSIS_TABLE = [
    # (condition_fn, message)
    (lambda: any(c == "HPC <-> ZCU" and s is False for z, c, s, d in results),
     "ZCU 전원 또는 이더넷 케이블을 확인하세요."),
    (lambda: (any(c == "HPC <-> ZCU" and s is True for z, c, s, d in results) and
              all(s is False for z, c, s, d in results if z != "L0" and c != "HPC <-> ZCU" and s is not None)),
     "ZCU 프로세스가 실행 중인지 확인하세요 (systemctl status zcu)."),
    (lambda: any("Camera" in c and s is False for z, c, s, d in results),
     "Camera RPi 전원 / cam.py 실행 상태를 확인하세요."),
    (lambda: any("ECU" in c and s is False for z, c, s, d in results),
     "ECU 전원 / CAN 케이블을 확인하세요."),
]


def print_diagnosis():
    hints = [msg for cond, msg in DIAGNOSIS_TABLE if cond()]
    if hints:
        print(f"\n{BOLD}Diagnosis:{RESET}")
        for h in hints:
            print(f"  -> {h}")


# ── Main ───────────────────────────────────────────────────────────────────

def main():
    print(f"\n{BOLD}{CYAN}AutoSDV Connectivity Diagnostic{RESET}")
    print("=" * 48)

    # L0
    if not check_l0():
        print(f"\n{RED}L0 FAILED: source /opt/ros/humble/setup.bash 실행 필요{RESET}")
        return EXIT_ENV_ERR

    # L1 + L3 per zone
    for zone_name, zone_config in ZONES.items():
        check_zone(zone_name, zone_config)

    # Summary
    print_summary()
    print_diagnosis()

    code = compute_exit_code()
    if code == EXIT_ALL_OK:
        print(f"\n{GREEN}All checks passed. System ready.{RESET}\n")
    return code


if __name__ == "__main__":
    sys.exit(main())
