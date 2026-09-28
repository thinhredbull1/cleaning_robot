                         ┌──────────────────────────────┐
                         │        AGV SYSTEM             │
                         └──────────────────────────────┘

        ┌──────────────────── MAP / LOCALIZATION ────────────────────┐
        │                                                            │
        │   ┌──────────┐      ┌──────────────┐      ┌────────────┐   │
        │   │ Mapping  │─────►│ Map Manager  │─────►│ Map Storage│   │
        │   │  SLAM    │      │              │      │   *.yaml   │   │
        │   └──────────┘      └──────────────┘      └────────────┘   │
        │        ▲                    ▲                              │
        │        │                    │                              │
        │   ┌──────────┐       Services                              │
        │   │Localization│      • start_mapping                      │
        │   │   AMCL    │       • stop                               │
        │   └─────┬────┘       • save_map                            │
        │         │             • start_localization                 │
        └─────────┼──────────────────────────────────────────────────┘
                  │ pose / map
                  ▼
       ┌────────────────────────────────────────────────────────────┐
       │                    NAVIGATION LAYER                         │
       │                                                            │
       │  ┌─────────────────┐                                      │
       │  │    move_base    │                                      │
       │  │ planner/costmap  │                                      │
       │  └────────┬────────┘                                      │
       │           │ trajectory / velocity command                 │
       │           ▼                                               │
       │  ┌─────────────────┐                                      │
       │  │  Path Tracking  │                                      │
       │  │   Controller    │                                      │
       │  └────────┬────────┘                                      │
       │           │                                               │
       │           └──────────────────┐                            │
       │                              │                            │
       │  Camera ──► Person Tracking ├────► MOVEMENT COMMAND       │
       │                              │                            │
       │  Dock Sensor/Pose ─► Docking┘                             │
       │                                                            │
       └──────────────────────────────┬─────────────────────────────┘
                                      │
                                      ▼
                         ┌─────────────────────────┐
                         │      moving_server      │
                         │                         │
                         │  Single Motion Gateway  │
                         │  & Command Arbitration  │
                         │                         │
                         │ ┌─────────────────────┐ │
                         │ │ Mode Manager        │ │
                         │ │ • MOVE_BASE         │ │
                         │ │ • TRACK_PERSON      │ │
                         │ │ • DOCKING           │ │
                         │ │ • STOP              │ │
                         │ └─────────────────────┘ │
                         │                         │
                         │ ┌─────────────────────┐ │
                         │ │ Safety / Validation │ │
                         │ │ • priority          │ │
                         │ │ • timeout           │ │
                         │ │ • velocity limit    │ │
                         │ │ • emergency stop    │ │
                         │ └─────────────────────┘ │
                         └────────────┬────────────┘
                                      │
                                  final cmd_vel
                                      │
                                      ▼
                         ┌─────────────────────────┐
                         │   Base Controller       │
                         │   Motor Driver          │
                         └────────────┬────────────┘
                                      │
                                      ▼
                                  ┌───────┐
                                  │ Wheels│
                                  └───────┘