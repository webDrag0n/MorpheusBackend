# MorpheusBackend

## Introduction

This is the ROS2 connector used to bridge controller code and unity. Need to be deployed on Linux based systems

## Deploy

- ## Docker
	- 下载`morpheus_backend.tar`
	- TODO 补充docker链接
	- TODO 需要将docker镜像名与文件名统一
	-
	  ```bash
	  docker load -i morpheus_backend.tar
	  docker run --name morpheus_backend --gpus all -it --shm-size=16g --rm -v /mnt/j/DockerDirs/morpheus/:/root -p 10000:10000 morpheus_backend:v1.2
	  [press ctrl-p-q to detach from container]
	  docker ps -a # get docker container id
	  docker exec -it [container_id] /bin/bash
	  
	  # activate humble environment
	  # IMPORTANT: CHECK IF ~/.bashrc ALREADY INCLUDES THE FOLLOWING COMMAND!
	  source /opt/ros/humble/setup.bash
	  echo " source /opt/ros/humble/setup.bash" >> ~/.bashrc
	  
	  # Get Morpheus backend repo
	  cd ~
	  git clone https://github.com/webDrag0n/MorpheusBackend.git
	  cd MorpheusBackend
	  # pull submodules such as ROS-TCP-Endpoint
   	  git submodule update --init --recursive
	  
	  # Build
	  colcon build
	  source install/setup.sh
          # Must do it twice
	  colcon build
	  source install/setup.sh
	  ```
- ## Build From Scratch

	- 首先使用[鱼香ROS](https://fishros.org.cn/forum/)教程安装ROS2-foxy，桌面版或基础版环境皆可，但是桌面版有更全面的功能，对于其他ROS2开发可能有用。
	- [一键安装教程](https://fishros.org.cn/forum/topic/20/%E5%B0%8F%E9%B1%BC%E7%9A%84%E4%B8%80%E9%94%AE%E5%AE%89%E8%A3%85%E7%B3%BB%E5%88%97)
	  ```bash
	  wget http://fishros.com/install -O fishros && . fishros
	  ```
	- 重启终端
	- 安装python3 em库
		- 如安装过程中报`No module named 'em'`错误，请通过`pip3 install empy==3.3.2`安装`em`模块，⚠️注意版本号不对也有可能报错，其他缺少模块报错只需缺什么装什么即可。
		-
		  ```bash
		  pip install empy==3.3.2
		  pip install catkin-pkg
		  pip install lark
    		  pip install pin
		  ```
	- 下载本仓库
		-
		  ```bash
		  git clone https://github.com/webDrag0n/Morpheus.git
		  # 拉取unity_robotics_demo_msgs
		  git submodule update --init --recursive
		  ```
	- 删除build目录（如有）并执行：
		-
		  ```bash
		  bash start_compile.sh
		  ```
	- 测试
		  ```bash
		  # conda 环境会扰乱ros2引用
		  conda deactivate
		  
		  ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
		  ```
		- 注意如果报错某package not found或import error，很有可能是没有一开始就关闭conda环境，可以尝试删除除src以外所有生成文件重新构建。
	- Unity端插件安装
		
		- 将项目目录下的MuJoCo.zip复制至用户目录（Windows：C:/Users/<UserName>）并解压，文件夹命名为`MuJoCo`
		- 安装ROS-TCP-Connector（Unity端插件）
		  - 在unity package manager中选择`install from git`并填入链接`https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
		- 安装URDF-Importer
		  - 官方教程：[Importing a Niryo One Robot using URDF Importer](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/urdf_importer/urdf_tutorial.md)
		  - 在unity package manager中选择`install from git`并填入链接`https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer#v0.5.2`，v0.5.2为版本号，请注意选择。
- ## 增加模块
	- 在`<workspace>/src/unity_robotics_demo/unity_robotics_demo/`目录下添加新publisher，如`h1_control_publisher.py`
	- 更改`<workspace>/src/unity_robotics_demo/setup.py`，在`entry_points`字段下以
	  ```python
	  entry_points={'console_scripts': ['h1_control_publisher = unity_robotics_demo.h1_control_publisher:main'], ...}
	  ```
	  的格式注册新模块  
	- 在`<workspace>/src/unity_robotics_demo/unity_robotics_demo_msgs/msg/`目录下添加新消息，如`H1ControlCommand.msg`
	- 更改`<workspace>/src/unity_robotics_demo/unity_robotics_demo_msgs/CMakeLists.txt`，在`rosidl_generate_interfaces`字段下以
	  ```cmake
	  rosidl_generate_interfaces(${PROJECT_NAME}
	  	"msg/H1ControlCommand.msg"
	      ...
	      "srv/xxx.srv"
	      DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
	  )
	  ```
	  格式注册新消息  
	- 最后执行`<workspace>/start_compile.sh`或
	  ```bash
	  cd <workspace>
	  source install/setup.bash
	  source build
	  source install/setup.bash
	  ```
	  完成编译  
- ## 运行
	- 运行前需要重新编译工作区，以防有更改在增加模块阶段忘记编译
	  ```bash
	  cd <workspace>
	  ./start_compile.sh
	  ./start_ros_tcp_endpoint.sh 
	  ./start_h1_publisher.sh
	  ```

## Tips

To git push from docker, it is better to set email and name locally in repo

```bash
git config --local user.email
git config --local user.name
```
