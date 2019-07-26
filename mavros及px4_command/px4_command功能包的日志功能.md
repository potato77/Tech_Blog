# px4_command功能包的日志功能

实验数据存储是一项很重要的功能，存储下来的数据可以进行日志分析或者绘图。

方式一：在机载电脑上进行实时存储。这种做法可能会占用机载电脑的cpu，对实时飞行性能造成影响。

方式二：在远程桌面笔记本上进行存储。
实现方式：ros多机+rosbag工具

## ros多机

转载来自：[ros多机](https://www.jianshu.com/p/69815d79d37f)

机载电脑IP：`192.168.31.107`
机载电脑HostName：`pi-desktop`

桌面电脑IP：`192.168.31.111`
桌面电脑HostName：`robot-desktop`

将机载电脑和桌面电脑连入同一局域网，相互ip可以通，因为未添加hostname ip映射，相互无法通过hostname联系。

修改环境变量，打开终端并输入

    gedit .bashrc


对于机载电脑(TX2等)
```
source /opt/ros/kinetic/setup.bash
LOCAL_IP=`hostname -I`
export ROS_IP=`echo $LOCAL_IP`
export ROS_HOSTNAME=`echo $LOCAL_IP`
export PIBOT_MODEL=apollo
export PIBOT_LIDAR=rplidar
export ROS_MASTER_URI=`echo http://$ROS_IP:11311`
```

对于桌面电脑
```
source /opt/ros/indigo/setup.bash
LOCAL_IP=`hostname -I`
export ROS_IP=`echo $LOCAL_IP`
export ROS_HOSTNAME=`echo $LOCAL_IP`
export PIBOT_MODEL=apollo
export PIBOT_LIDAR=rplidar
export ROS_MASTER_URI=`echo http://xxx.xxx.xx.xx:11311`
source ~/pibot_ros/ros_ws/devel/setup.bash
```
  
xxx.xxx.xx.xx为机载电脑IP

ros多机是一个主从模式，一个为主机，其余从机连接主机的master节点，故从机需要添加主机的IP地址。

多机通讯只能有一个master存在，所有都设置`ROS_MASTER_URI`为`http://MASTER_IP:1131`, 各个终端要能相互通讯就必须设置自己的ROS_IP

## rosbag工具

存储所有话题

    rosbag record -a

存储指定话题

    rosbag record -O subset /turtle1/cmd_vel /turtle1/pose

其他指令

    rosbag info <your bagfile>
    
    rosbag play <your bagfile>


## rosbag与Matlab

直接使用Matalab打开rosbag文件

[教程](https://www.mathworks.com/help/robotics/ref/rosbag.html)

    bag = rosbag('ros_turtlesim.bag');
    bSel = select(bag,'Topic','/turtle1/pose');

Read messages as a structure. Specify the  `DataFormat`  name-value pair when reading the messages. Inspect the first structure in the returned cell array of structures.

    msgStructs = readMessages(bSel,'DataFormat','struct');
    msgStructs{1}

现将bag文件存储为txt格式，再通过Matlab读入数据

    rostopic echo -b file_name.bag -p /topic_name > Txt_name.txt
