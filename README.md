# Rigid Body State Estimation 

## Installation
```cmd
cd [ROS_workspace]/src
git clone https://gitee.com/hai_zhu/state_estimator
cd ..
catkin_make
source devel/setup.bash
```

## State estimation using Mocap
- Change the parameters in the `.yaml` file within the `config` directory. 
- Feed the `.yaml` file to the `.launch` file and check the `remap` lines to make topic names matched.
- Launch the node
  ```cmd
  roslaunch state_estimator mocap_state_kf_multiple.launch
  ```
