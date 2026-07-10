# AutoSDV Shared Documentation

This folder contains shared project plans and documentation for the AutoSDV project.

## Structure

```
.claude/
├── README.md                    ← This file
├── plans/                       ← Implementation plans
│   ├── session-plan-ko.md       ← Master plan (overall progress)
│   ├── zcu-optimization-plan.md ← ZCU code issues (18 items)
│   ├── ecu-optimization-plan.md ← ECU code issues (7 items)
│   └── diagnostic-plan.md       ← Connectivity diagnostic tool
└── (architecture.md)            ← TODO: Project + HW/network overview
```

## Plan Files

| File | Description | Status |
|------|-------------|--------|
| session-plan-ko.md | Overall session plan (master) | Active |
| zcu-optimization-plan.md | ZCU S32G3 code optimization (18+6 issues) | 17/18 done + 6 new |
| ecu-optimization-plan.md | ECU RA6M5 code optimization (7+3 issues) | Not started |
| diagnostic-plan.md | HPC connectivity diagnostic tool (L0+L1+L3) | Planned |

## How to Use

1. Read `plans/session-plan-ko.md` for overall status
2. Check module-specific plan for details
3. After implementing, update the plan status
4. Project rules are in `/AutoSDV/CLAUDE.md`
