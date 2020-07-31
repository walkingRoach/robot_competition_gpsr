# 中国机器人大赛通用差速模型
## 前言
中国机器人大赛服务机器人2019年通用赛事一等奖代码。通过模块式的整合开发,实现在室内环境中差速模型小车的定位、避障、导航为目标。  
具体实现的功能有：
1. 机器人建图、定位、导航
2. 物体识别与抓取
3. 人机语音交互（支持日常常规简单语句的交流和中国机器人大赛中指定的交流）
4. 人体跟踪
5. 远程语音电器控制(使用的串口协议)
6. 人脸识别与辨识

## 内容列表
- [模块划分](#模块划分)
- [安装](#安装)
- [使用](#使用)
- [后记](#后记) 

## 模块划分
初步代码模块分为sensor(传感器数据模块)，navigation(包含建图、导航、定位模块)， perception(包含视觉处理模块)，communication（人机通讯模块), robot(通用模块), car_model(车辆urdf模块)。  
注意的是这里只分大模块，不分更加具体的小模块。
接下来会对以上模块进行详解结构介绍。
### 1.sensor(传感器模块)
用于对传感器数据进行获取和处理。目前包含有
- laser (ydlidar)
- camera (D435, D435i)
- motor (自制底盘通信)
- arm (自制机械臂)

### 2.navigation模块
用于对环境建图，对地图处理，机器人定位，机器人路径规划，路径跟踪这几大模块进行综合处理
- map_generator(地图构建, gmapping)
- localization(amcl定位)
- nav(基于move_base的导航算法)

### 3.perception(视觉处理模块)
- face_detector(人脸辨识，facenet)
- object_detector(物体识别, darknet(yolov3))
- tacker(动态物体跟踪, kcf, pysot)

### 4.communication(无线通信模块)
- xf_ros (科大讯飞语音识别)

### 5. robot(核心通用模块)
- AIML(固定语音回复,选用不同的aiml文件用以支持不同类型的交流)
- face_datasets(对比人脸库，使用名字加序号来命名)
- image(人脸，物体检测结果保存图像)
- models(神经网络模型文件, 接近废弃)
- wav(预先生成常规对话wav语音文件)
- src(核心通用实现)
    - body:实现核心, 暂时只开源config(机器人全局配置文件), control(串口远程控制电器), walk(机器人安坐标，地址名导航)
    - gpsr_sm: 整个gpsr项目的逻辑实现状态机。
    - test: 单元测试模块。

### 5.vehicle(车身模型和传感器模型模块)
- wrcar_description(自身用车辆urdf)

## 安装
自己解决安装。提示作为本人早期项目，包中的编译依赖关系并没有做好, 建议各个包单独编译。

## 使用
完整运行程序对电脑要求较高,运行时需要使用多达4个神经网络，大约消耗显存4g左右。如果发现节点突发状态退出，大概率需要提高电脑配置
### 1. 建图
```bash
# 注意修改其中的传感器设置
roslaunch map_gmapping create_map.launch

# 启动控制脚本
rosrun mav_ros keyboard_teleop.py

# 保存地图
rosrun map_server map_save -f -map_name
```
### 2. 导航
```bash
roslaunch nav_ros nav.launch
```

### 3. 模块单元测试
由于基础框架是使用python进行编写，所以单元测试必不可少,所有的测试项都放在/src/test文件中
```bash
# 例如进行语音测试
roslaunch robot test_mouth_and_ear.launch
```

### 4. 整个机器人gpsr项目
```bash
tmux 

./run/gpsr.zsh
```
## 后记
鉴于github并没有特别好的中文的中国机器人大赛的参考实现，希望参加机器人大赛的人一开始就能有好的学习参考(仅仅是参考，不要照抄一切)。本文各个模块的功能实现或许不够出色，但是整体结构上是可以借鉴的。个人认为模块化的编程思路在应对机器人这样复杂的机构上尤为的重要。在整个设计模式也仅仅是基于简单的外观，策略模式。是否进一步解耦，设计更好的模块结构也在于未来的人的实现。如果有更好的想法，可以联系我(QQ:1145893246)
