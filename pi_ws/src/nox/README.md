# FLOW:
1. ServerInter.py --> Parse waypoint from server. --> Get to moving server. --> Also get localization data and status for publish state.
2. MovingServer.py --> Get waypoint from server --> publish DOING msg for hardwareInterface --> wait MOVING msg for move_base start -->  when follow waypoint success -> publish SUCCESS.
3. HardwareInterface.py -> First in WAITING MODE --> wait DOING msg MovingServer --> wait arduino receive for lock or unlock or module ... --> arduino send "OK " --> change to DOING Mode --> Send MOVING msg to MovingServer for start move_base --> WAITING for msg SUCCESS --> change to SUCCESS mode --> send to arduino,sim and wait response --> done.
4.CMD vel --> esp32

## IMPLEMENT step: 
1. Get encoder and odometry working with pubvelencoder --> Need esp32 code and pubvelencoder --> 1-2 day
2. State estimation fusion with EKF for odom and IMU only --> robot_localiztion package --> lidar package for check result --> 2-3 day
3. Navigation without map with EKF (odom and IMU ) --> checking waypoint process and path tracking --> 1-2 day
4. GPS  fusing robot localization , sim integrate- 1-2 day.
5. Server interface fusion --> 1-2 day.
6. Arduino output integrate --> 1 day.
7. Total test --> ...
---> 11 day to reach total test.