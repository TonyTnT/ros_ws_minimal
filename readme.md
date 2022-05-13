# ros环境配置
一键安装指令
```
wget http://fishros.com/install -O fishros && . fishros
```
# 功能包环境配置

```
cd ros_ws_minimal
# 安装 src下功能包所需依赖
rosdep install --from-paths src --ignore-src -r -y
````

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

