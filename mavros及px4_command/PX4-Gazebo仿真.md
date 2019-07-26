# 【教程】PX4-Gazebo仿真

今天，给大家介绍下PX4开源飞控项目中一个强大的功能-Gazebo仿真。

Gazebo仿真是PX4提供众多仿真工具中的一个。它在PX4代码中（Firmware/Tools/ **[sitl_gazebo](https://github.com/PX4/sitl_gazebo)**）也是一个单独出来的仓库。


## PX4中都有哪些仿真工具？他们都有什么区别？

官方介绍请移步：[PX4-开发手册-仿真](https://dev.px4.io/en/simulation/)

仿真首先分为软件在环仿真（SITL）和硬件在环仿真（HITL）。目前来看，软件在环仿真更简单实现及方便。我就读的课题组就是专门做导弹的半实物仿真的，个人认为硬件在环仿真需要加上转台才能真正意义发挥出作用，不然只是在Pixhawk板子上跑仿真环境，毫无指导意义。

软件在环仿真一共是有[jMAVSim](https://dev.px4.io/en/simulation/jmavsim.html)、[Gazebo](https://dev.px4.io/en/simulation/gazebo.html)、[AirSim](https://dev.px4.io/en/simulation/airsim.html)这三种。jMAVSim是一个轻量级的仿真器，目前只支持四旋翼仿真。AirSim我不太清楚，没有使用过，这里就不评价了。Gazebo是我们今天的主角，支持旋翼、固定翼、倾转、小车等，是所有仿真器里支持平台最多的，也能支持多个无人机的仿真，在各个仿真器比较的表格里，PX4官方是这么说Gazebo仿真的：**This simulator is highly recommended.**

一般而言，如果我是修改了PX4固件内的代码，比如修改了姿态控制器，我会用jMAVSim调试，同时打开地面站，利用定点及自稳模式进行飞行测试，还能下载log看看记录的量对不对。jMAVSim不吃电脑配置，运行比较流畅，适合快速验证PX4内部代码逻辑及检查修改固件后的BUG。

如果我需要用到px4_command及mavros包来进行offboard模式的测试，我会使用Gazebo仿真。比如我在机载电脑中修改了一些控制逻辑，打开Gazebo仿真，同时运行mavros及相应节点，将仿真的无人机切换至offboard模式，在Gazebo中测试我修改的代码是否正确，十分好用！

这只是我个人的使用习惯，正常来讲，我后面说的那个功能用jMAVSim也能做，但你既然都跑ROS了，肯定用一个和ROS相关的仿真器更加好用一点。jMAVSim比不过Gazebo的一点是它无法进行固定翼、小车的仿真（但我也没试过），以及无法进行视觉类的仿真，无法修改飞行环境等等。具体Gazebo还能做什么，可以自行百度，或参阅[Gazebo官网](http://gazebosim.org/)，毕竟光学Gazebo就够一个人学一段时间的了（这点就和ROS一样，但放到我们场景中来说，还是你需要什么就学什么，不然你永远都学不完）。

总而言之，Gazebo仿真功能强大，值得推荐！

## PX4的仿真是如何进行的？
![enter image description here](https://dev.px4.io/assets/simulation/px4_simulator_messages.png)

上一张官方图，Flight stack代表飞控即PX4，Simulator代表仿真器（如Gazebo）。所有仿真器与PX4的通讯都是通过MAVLink消息来进行的，SITL使用simulator模块中的[simulator_mavlink.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/simulator/simulator_mavlink.cpp)来处理这些消息，而HITL是使用mavlink模块中的[mavlink_receiver.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp)来处理这些消息。梳理一下这里的消息流向：

 1. **PX4到仿真器**。PX4给仿真器只会发送一个[HIL_ACTUATOR_CONTROLS](https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS)的MAVLink消息，熟悉PX4的应该知道这个对应的uORB消息是[actuator_outputs.msg](https://github.com/PX4/Firmware/blob/master/msg/actuator_controls.msg)，也就是姿态控制器最后的输出控制量。这里也就意味着，混控是在仿真器中进行的，仿真器中也包含电机的模型。
 2. **仿真器到PX4**。仿真器的作用就是模拟真实飞行，即模拟计算出真实飞行时的传感器状态，包括GPS，IMU等，将这些信息发送给飞控后，再由飞控中的估计模块计算出飞机状态量。
 3. **外部到PX4**。这里的外部就比如地面站QGC（可以外接遥控器），Mavros，Dronecode SDK等，这里也就可以模拟我们平时控制飞机的方式。
 
 下面这张图更清楚的表示出了PX4与各个部分之间的关系。仿真器包含了传感器和执行机构的模型，负责产生传感器的原始数据和执行最后的底层控制指令；Offboard API及QGC负责发送传输上层指令（位置期望点、速度期望值等）并且监控飞机状态；PX4固件本身则运行控制及估计模块（即飞控核心算法）。

![enter image description here](https://dev.px4.io/assets/simulation/px4_sitl_overview.png)

**端口号**
PX4是使用UDP来进行这些消息通讯的。单个飞机仿真时，默认的UDP设置如下：
-   UDP端口  **14540**  用于offboard模式的通讯。is used for communication with offboard APIs. Offboard APIs are expected to listen for connections on this port.
-   UDP端口  **14550**  用地面站的通讯。
-   TCP端口 **4560**  用于与仿真器的通讯。 PX4 listens to this port, and simulators are expected to initiate the communication by broadcasting data to this port.

如果不涉及多个飞机的仿真，端口号默认都是设置好的，不需要修改及配置。如果是多个飞机的仿真，则需要配置每个飞机的端口号，不能重复，后续会详细介绍配置。

上述介绍适用所有的仿真器。

## Gazebo仿真

环境搭建我这里基本就是照搬+翻译PX4的开发手册了，如有疑惑，请留言或移步[手册](https://dev.px4.io/en/simulation/)。还是和以前的看法一致，千万不要百度，里面的解答大部分都是带着你走歪路。

首先，你要成功搭建PX4固件开发环境，你要能成功编译PX4代码。（这一步都没成功，那就先搞好这一步）如果你在搭建PX4环境时，没有顺带装一下Gazebo，可能需要单独安装（打开终端输入gazebo可以检测你的电脑是否安装gazebo）。

下面的内容我将分成非ROS下的Gazebo仿真和ROS下的Gazebo仿真。个人是推荐后者的，如果你不使用ROS，也没必要上Gazebo仿真了。

可以发现，在PX4固件中有一个文件夹`Firmware/Tools`这里面就是包含了jMAVSim、Gazebo仿真等一系列代码。而Gazebo仿真对应的文件夹是`sitl_gazebo`，这其实是一个子库，即[sitl_gazebo](https://github.com/PX4/sitl_gazebo/tree/f2593dbcc2980dd15714925931fbb0e46a63a4b2)这个git仓库。

大致看看这个文件夹里都有些什么内容。
 - src文件夹。这里是核心代码，即plugin（插件）代码。比如gps、imu、电机的插件，简单说如何模拟产生gps信号，就是这里的代码负责的。Gazebo自身也会提供一些默认插件，或者第三方也会提供插件，比如你要新增一个传感器，就可以找到相应的插件。
 - models文件夹。存放各种Gazebo模型文件。
 - worlds文件夹。存放Gazebo的世界环境文件。

个人认为涉及到Gazebo仿真二次开发就是如上三个文件夹（如果你只是使用Gazebo仿真工具，这里你什么都不需要修改，因为已经完全搭建好了），分别对应修改模型、修改模型参数、修改世界。

## 非ROS下的Gazebo仿真

按照PX4手册中的[Gazebo仿真](https://dev.px4.io/en/simulation/gazebo.html)教程，进行四旋翼仿真应该是

     cd ~/src/Firmware
     make px4_sitl gazebo

我的固件用这个指令失败了，但其他小伙伴执行这一步却正常运行了。原因暂不清楚，这里待补充。



## ROS下的Gazebo仿真
除了上述的教程，在PX4开发手册中，还有一个属于Gazebo仿真的教程：[ROS with Gazebo Simulation](https://dev.px4.io/en/simulation/ros_interface.html)。这就是指导大家如何在ROS环境下，联合Mavros功能包进行仿真调试。我的固件按照这个教程一步一步走下来，没有任何问题。（PX4项目太大，各个部件分散，也不是商业项目，坑还是多~）

对于ROS、Mavros、Offboard模式还不清楚的小伙伴，先去补补相关概念，这里就不多做介绍了。

**注意**：第一步和第二步只需要做一次，设置成功后，每次直接执行第三步。但如果修改了PX4固件，那你需要重新执行第一步。

第一步：**编译**。

     cd ~/src/Firmware
     make px4_sitl_default gazebo

第二步：**source**。了解ros功能包应该知道这一步是为了让系统知道有这个功能包存在，相当于设置环境变量，这样终端执行的时候就不会报找不到功能包的错误。

    source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

推荐大家手动source，打开终端输入`gedit .bashrc`。在弹出的txt文件最后一行，加入上述内容。值得注意的是，`$(pwd)`是要替换成对应的目录路径（PX4固件所在目录），我的路径是`/home/fly_vision/px4_amov/Firmware`（fly_vision是我的电脑名字，px4_amov是我存放固件的文件夹名字）。于是我的修改内容如下：

    source /home/fly_vision/px4_amov/Firmware/Tools/setup_gazebo.bash /home/fly_vision/px4_amov/Firmware /home/fly_vision/px4_amov/Firmware/build/posix_sitl_default 
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/fly_vision/px4_amov/Firmware
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/fly_vision/px4_amov/Firmware/Tools/sitl_gazebo

第三步：**运行Gazebo仿真**
```
roslaunch px4 posix_sitl.launch
```
因为会运行Gazebo，第一次启动会比较慢，耐心等待（如果报错，请关闭终端再次尝试，我的i5笔记本经常启动失败，这是你的笔记本提示你该花钱了的正常现象）。启动成功后，会看到Gazebo中有一个iris无人机。这时你可以打开QGC地面站，地面站会默认连接这台飞机，你可以尝试利用地面站发送起飞指令测试。

这个启动脚本位于`Firmware/launch`文件夹中，同时在此文件夹中，你还能看到其他类似的启动脚本。这里建议大家单独启动Mavros，而不是用它提供的另一个启动脚本同时启动（因为一个终端显示太多东西的话，报错你都看不到）。

第四步：**运行Mavros**
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```
此处我建议Mavros功能包使用二进制安装，比较省事。可以看到这里启动mavros时配置了一些端口参数（端口匹配才能成功连接），后续我会在PX4仿真模块中找到与之对应的参数配置。

尝试读取飞空的IMU数据，打开终端，输入

    rostopic echo /mavros/imu/data

当然，除去imu消息，还有很多其他消息，可以使用下面指令查看都有哪些消息。

    rostopic list

第四步：**运行px4_command**

px4_command功能包是阿木社区最新推出的科研无人机P200中的核心代码，暂时未开源（后续会开源）。这里待补充。

目前，我利用Gazebo仿真复现了px4_command功能包中的除去视觉之外的绝大部分功能，比如惯性系移动、机体系移动、悬停、降落等控制逻辑。除此之外，重新设计了位置控制器，设计了基于UDE的外环控制器，在仿真环境中进行了测试。总体用下来，感觉Gazebo仿真是进行二次开发的利器。比如，我修改了一些px4_command中控制无人机的上层代码，以前我只能通过真实飞行进行测试，那样有炸鸡危险。现在，我只需要使用Gazebo仿真，模拟真实飞行中的命令，在仿真环境中查看飞机的表现。值得注意的是，目前还只能检查代码的逻辑错误，及发送给飞控指令是否正确。自然不能100%仿真复现真实飞行中的轨迹，这是由于飞机的模型，传感器的模型，执行机构的模型都不是你真实飞行时的模型。




## 多机Gazebo仿真

在单个飞机仿真没问题之后，才能执行这一步。
```
roslaunch px4 multi_uav_mavros_sitl.launch
```
这是官方提供的多机仿真启动脚本，只提供了两架飞机，现在给出如何增加一台飞机的方法。

-    在启动脚本中 **multi_uav_mavros_sitl.launch**增加 `UAV3`
     -   复制一个组 (`UAV1`  or  `UAV2`)
     -   将  `ID` 这个参数设置为  `3`
     -   为  `mavlink_udp_port`  这个参数选择一个新的端口号，用于与Gazebo通信。
     -   修改`fcu_url`  这个参数，用于与mavros通信。
-   创建一个新的rcS启动文件:
  
    -   从启动文件中复制一个 (`iris_1`  or  `iris_2`)，并重命名为  `iris_3`。
    -   `MAV_SYS_ID`  修改为  `3`；
    -   `SITL_UDP_PRT`  值要与位于启动脚本中  `mavlink_udp_port` 参数匹配；
    -   第一个  `mavlink start`  端口号要与 `mavlink stream`  端口号一致，用于与QGC通信。
    -   第二个 `mavlink start` 端口号要与启动脚本中 `fcu_url` 参数一致。

这里比较混乱，建议大家亲手做一遍，加深理解。但是，我也会分享我修改后的两个文件给大家参考。
