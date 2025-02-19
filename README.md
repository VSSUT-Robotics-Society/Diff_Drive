## Differential Drive Bot
This repo contains the necessary ROS files required for building a Differential Drive Bot.

- To Run the Gazebo Sim: 
```ros2 launch mangalyaan sim.launch.py```

- To Run the Real Robot: 
```ros2 launch mangalyaan bot.launch.py```

- To Run/Check URDF Files: 
```ros2 launch mangalyaan urdf.launch.py```

- For Rviz: 
```rviz2 -d "~path/to/the/file.rviz"```

- For Teleop: 
```ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped```
