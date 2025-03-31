# g1 控制接口

## 确认是否可以下肢服务控制，上肢电机控制，在什么模式下
    上肢控制可以在运行机器人内置移动控制器的同时，单独控制上肢电机完成操作任务，提供 DDS 接口
    DDS 接口支持上肢控制，仅能在锁定站立、运控 1 与运控 2 中使用
    运控 1 与运控 2 是G1的两种运动步态，你可以遥控R1+X（主运控），R2+X（跑步运控）进行切换这两个运控体验一下


## 使用服务功能驱动下肢

    1、进入高层控制模式
        1、进入阻尼模式：长按 L1 + 单击 A
        2、将机器人脚着地
        3、进入锁定站立模式：长按 L1 + 单击 UP
        4、进入运控1: R1+X（主运控）
        5、或进入运控2：R2+X（跑步运控）

    2、构建ros2节点
        cd ~/unitree_ros2/unitree_ros2_ws/ # 进入unitree_ros2工作空间
        colcon build #编译工作空间下的所有功能包

    3、测试连接性
        ros2 topic list

    4、测试loco服务例程
        ./install/unitree_ros2_example/bin/g1_loco_client_example --move="0 0 0.5"


    5、如果可行，python改造（保存到github）
        
        这是一个服务通信模式，还要该写base_client.hpp文件

# 传感器驱动安装
### 安装nomachine
    1、下载安装包
    https://downloads.nomachine.com/download/?id=1

    2、然后ssh进入pc2并./nomachine.sh运行脚本文件
    3、然后打开自己pc的nomachine就可以看到pc2里面的图形化界面了

### 连接到同一个wifi
    将pc2和自己的pc连到同一个wifi
    然后用自己的pc ssh进入wifi给pc2分配的ip就可以无线开发了


## 雷达驱动安装到PC2

    1、下载ros2驱动
        git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2

    2、安装Cmake，g++
        sudo apt update
        sudo apt install cmake
        cmake --version


    3、下载SDK
        git clone https://github.com/Livox-SDK/Livox-SDK2.git
        cd ./Livox-SDK2/
        mkdir build
        cd build
        cmake .. && make -j
        sudo make install

    4、ip地址修改
        需要修改.../livox_ros_driver2/config/MID360_config.json 文件内的 "lidar_configs" "ip" 为 "192.168.123.120"
        "host_net_info" 下的 ip 为主机 ip——（G1的pc2：192.168.123.164，笔记本电脑：192.168.123.145）

    5、构建ros2驱动
        source /opt/ros/foxy/setup.sh
        ./build.sh ROS2

    5、运行驱动
        5.1、通过rviz显示雷达点云
        ros2 launch livox_ros_driver2 rviz_MID360_launch.py

        5.2、通过/livox/lidar(点云话题）、/livox/imu话题，订阅数据
        ros2 launch livox_ros_driver2 msg_MID360_launch.py



## 相机驱动（已安装）

    1、启动节点
        
        ros2 launch realsense2_camera rs_launch.py
        ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true

        
        默认值：
        camera_namespace:=camera
        camera_name:=camera   

    2、通过添加以下参数来启用加速度和陀螺仪
        ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true enable_gyro:=true enable_accel:=true

    3、获取话题和服务
        > ros2 topic list
            /camera/camera/color/camera_info
            /camera/camera/color/image_raw
            /camera/camera/color/metadata
            /camera/camera/depth/camera_info
            /camera/camera/depth/image_rect_raw
            /camera/camera/depth/metadata
            /camera/camera/extrinsics/depth_to_color
            /camera/camera/imu

        > ros2 service list
            /camera/camera/device_info

# 导航建图方案


# 在go2上测试