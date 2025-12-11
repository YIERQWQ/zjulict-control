# 运行说明
## 环境
ubuntu 22.04 + ros2 humble + gazebo fortress

## 步骤
1. 克隆项目并切换到simulation_with_odom分支
2. 编译：在zjunlict文件夹下执行 colcon build
   - 文件结构：zjunlict下包含build、install、log、src
3. 运行：
   - 终端1（启动仿真器）：
     source install/setup.bash
     ros2 launch robot gazebo.launch.py
   - 终端2（查看数据）：
     source install/setup.bash
     ros2 run robot monitor
   - 终端3（速度控制）：
     source install/setup.bash
     ros2 run robot test_motion

## 说明
小车为理想模型，完全跟随速度指令运动；IMU和odom数据无误差，可按需添加。
