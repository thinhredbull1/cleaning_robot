cmd_vel mux package with priority config 


navigation ──────► /cmd_vel/navi ──┐
joystick ────────► /cmd_vel/joystick├──► cmd_vel_mux ──► /cmd_vel
tracking ────────► /cmd_vel/tracking│
docking ──────────► /cmd_vel/docking │
emergency ───────► /cmd_vel/emergency┘