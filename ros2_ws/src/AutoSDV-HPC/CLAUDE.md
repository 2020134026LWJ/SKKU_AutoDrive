# AutoSDV Project Rules

## Project Overview
1/5 Scale SDV(Software-Defined Vehicle) platform based on Zonal Architecture.
SKKU Graduate thesis project.

## Code Rules
- Before modifying any source file, create a backup: `[name]_bak.[ext]`
- ZCU settings are in `zcu_config.h` only (no hardcoding in source files)
- CAN IDs must use defines from config headers (CAN_STEERING_ID, CAN_SPEED_ID, CAN_STATUS_ID)
- `/octo:plan` = create plan files only (no code changes)
- `/octo:develop` = implement code changes

## Commit Message Convention
Use module tags:
- `[ZCU]` - S32G3_ZCU code changes
- `[ECU]` - RA6M5_ECU code changes
- `[HPC]` - Laptop_HPC code changes
- `[CAM]` - Rpi_CameraModule code changes
- `[SVC]` - Etc/ systemd service changes
- `[DOC]` - Documentation/plan changes
- `[DIAG]` - Diagnostic tool changes

Example: `[ZCU] Fix race condition in global CAN variables (#1)`

## Plan Files
- Plans are stored in `.claude/plans/`
- Create plan before implementing (`/octo:plan`)
- Update plan status after implementation

## Auto Plan Update Rule
When you complete an implementation task (code change), you MUST:
1. Find the related issue in `.claude/plans/` files
2. Update its status from ❌ to ✅ with a brief description of what was done
3. Update `session-plan-ko.md` if overall phase status changed
This ensures plan files always reflect current progress.

## System Architecture (DO NOT misunderstand)
```
Camera(RPi) → UDP → ZCU(S32G3/FastDDS) → DDS → HPC(Laptop/ROS2)
HPC → DDS → ZCU → CAN → ECU(RA6M5/FreeRTOS)
ECU → CAN → ZCU → DDS → HPC (status feedback)
```
- Camera does NOT communicate directly with HPC
- ZCU is the gateway between Camera/ECU and HPC
- ZCU uses FastDDS (not ROS2), HPC uses ROS2 (rmw_fastrtps)
- HPC can only access 10.0.0.x network (not 11.0.0.x/12.0.0.x)

## Network Reference
- HPC: 10.0.0.10
- ZCU Zone1: 10.0.0.2 (Steering)
- ZCU Zone2: 10.0.0.3 (Front_Left)
- ZCU Zone3: 10.0.0.4 (Front_Right)
- ZCU Zone4: 10.0.0.5 (Rear)
- Camera networks: 11.0.0.x, 12.0.0.x (ZCU-only, not accessible from HPC)

## Repository Structure
Each module has its own git repo:
- `S32G3_ZCU/` - Zone Controller (NXP S32G3, FastDDS, C/C++)
- `RA6M5_ECU/` - Motor ECU (Renesas RA6M5, FreeRTOS, C)
- `Laptop_HPC/` - Central compute (ROS2, Python/C++)
- `Rpi_CameraModule/` - Camera modules (Raspberry Pi, Python)
- `Etc/` - systemd service files
- `.claude/` - Shared plans and documentation
