# Daily commands

## Laptop

```bash
cd /home/fsti/ROS/TurtleBot
python3 HardwareScripts/start_robot.py --lidar
python3 DataCollection/record_bag.py --duration 20 --bag-name stationary_01
python3 NavigationScripts/nav_and_collect_v2.py --navigate --distance 1 --speed 0.1 --bag-name walk_01
python3 DataCollection/audit_bag.py walk_01
python3 HardwareScripts/stop_robot.py
```

Do not run all commands blindly: the navigation command moves the robot. Starting drivers and recording-only do not send nonzero navigation commands. Bags copy to the laptop automatically; use `--no-fetch` and `DataCollection/fetch_bag.py <name>` for deferred copying. Audit on the robot before fetching if the report should be included. Laptop source of Jazzy is not needed for these SSH launchers.

## Robot, native launch

```bash
ssh ubuntu@10.130.58.6
source ~/MA_Raihan/env.sh
ros2 launch ma_turtlebot_bringup robot.launch.py lidar:=true camera:=true
```

This foreground launch replaces the background start wrapper; do not launch both. Stop it with Ctrl+C. Alternatively `bash ~/MA_Raihan/start.sh --lidar` starts it in the background, and `bash ~/MA_Raihan/stop.sh` stops our tracked processes.

## Robot, direct experiment

```bash
bash ~/MA_Raihan/run.sh record --duration 20 --bag-name stationary_02
bash ~/MA_Raihan/run.sh navigate --distance 1 --speed 0.1 --bag-name walk_02
bash ~/MA_Raihan/audit.sh walk_02
```

These direct commands keep the bag on the robot. Use the laptop fetch helper afterward. See README.md for limits, required sensors, frame/timestamp semantics, and optional RealSense preparation.


## Updated Waffle Pi speed limit

Maximum configured translation is 0.26 m/s, matching the ROBOTIS Waffle Pi specification: https://emanual.robotis.com/docs/en/platform/turtlebot3/features/ . Turning remains limited to 0.8 rad/s. Defaults remain unchanged.

Keyboard driving without recording:
```bash
python3 NavigationScripts/teleop_v2.py --speed 0.26 --turn-speed 0.5
```
Navigation with recording:
```bash
python3 NavigationScripts/nav_and_collect_v2.py --navigate --distance 1 --speed 0.26 --audit
```
The robot command watchdog must be restarted after deploying a changed limit.


## Offline bag insights

No ROS sourcing or robot connection is needed:
```bash
python3 /home/fsti/ROS/TurtleBot/DataCollection/bag_insights.py /home/fsti/ROS/TurtleBot/Bags/walk_01
```
Replace the bag path as needed. This command prints the report directly in the terminal and creates no report files. For HTML, run `python3 DataCollection/bag_insights_html.py Bags/walk_01`; that version saves HTML, Markdown and JSON under `Insights/`. Reports include actual bag topic counts, source/receipt rates, timing gaps, camera-to-sensor nearest timestamps, and saved navigation/audit metadata. Close timestamps do not prove hardware synchronization. This does not modify the original database or start robot nodes.
