# ros环境配置
一键安装ROS指令  如果已有ros环境 跳过这步
```
wget http://fishros.com/install -O fishros && . fishros
```
会安装ros环境到全局
# 功能包环境配置
```
cd ros_ws_minimal
# 安装 src下功能包所需依赖
rosdep install --from-paths src --ignore-src -r -y
````
会将当前ros_ws_minimal中所有ros功能包依赖安装到全局


libuvc 安装
```
# ros libuvc
sudo apt install ros-$ROS_DISTRO-libuvc-*
# libuvc https://github.com/libuvc/libuvc
git clone https://github.com/libuvc/libuvc
cd libuvc
mkdir build
cd build
cmake ..
make && sudo make install
```
安装完成后显示
![Img](https://gitee.com/chenx2ovo/picgo2/raw/master/202205171022483.png)

要求 libuvc 库路径与cmake中一致
![Img](https://gitee.com/chenx2ovo/picgo2/raw/master/202205171023337.png)



# 功能包编译
```
catkin_make
source devel/setup.bash
```

# 导航功能
```
roslaunch xtark_nav xtark_nav.launch map_file:=xxxxx
```

常用参数
| param_name | default |
| use_realsense | false |
| use_gemini | true |
| -- | -- |
{.small}

