# MAVLink协议

## 目录

- **前言**
- **MAVLink 2协议介绍**
  - **MAVLink 2协议内容**
  - **MAVLink 2签名机制**
- **安装与入门**
  - **安装MAVLink及生成MAVLink库**
  - **使用MAVLink库**
  - **MAVLink消息收发的源码解读**
- **Demo源码**

## 前言

MAVLink全称是(**Micro Air Vehicle Message Marshalling Library**)，是一种轻量级的消息传输协议, 用于无人机之间（或机载组件之间）的通信。

消息通过XML文件进行定义。每个XML文件对应一个特定的MAVLink协议消息系统，被称为“dialect”（其实就是自定义的消息集，PX4中使用就是MAVLink官方提供的common消息集）。

MAVLink工具链可以通过指定的XML文件生成支持多种编程语言（C、C++、python，java等11种）的MAVLink库。常见的地面站（QGC、MP等）、飞控（PX4、APM等）及其他MAVLink系统（mavros、MAVSDK等）均是使用生成得到的MAVLink库进行的开发。MAVLink及其相关源码均遵循MIT协议，即可以无限制的在闭源应用中使用。

> 基于C语言封装的MAVLink库，是一个只有h文件的库，其针对资源受限的硬件进行了高度优化，并已成功部署在许多产品中。



MAVLink由Lorenz Meier在2009年早期发布，目前由众多开发者共同维护。在此，感谢Lorenz大神及其他代码开发者，并附上和Lorenz的合照一张。

**相关链接**：

- MAVLink开发手册
  - 链接：https://mavlink.io/
  - 有能力尽量阅读英文原文，因为中文翻译存在滞后或者偏差的情况
- MAVLink Github主页
  - 链接：https://github.com/mavlink
  - qgroundcontrol、mavros、MAVSDK的仓库都在这里，当然还有mavlink库本身，还有一些示例代码

**注意**：一切以官方文档和代码为准，由于代码更迭较快，本文有一定时效性！



## MAVLink 2协议介绍

### MAVLink 2协议内容

具体内容如下图及表格：

![MAVLink v2 packet](https://mavlink.io/assets/packets/packet_mavlink_v2.jpg)

| Byte Index       | C version                  | Content                              | Value        | Explanation                                                  |
| ---------------- | :------------------------- | ------------------------------------ | ------------ | ------------------------------------------------------------ |
| 0                | `uint8_t magic`            |                                      | 0xFD         | 起始帧头                                                     |
| 1                | `uint8_t len`              | Payload length                       | 0 - 255      | `payload` 长度                                               |
| 2                | `uint8_t incompat_flags`   | 签名标志位                           |              | 是否签名，1代表本消息已签名                                  |
| 3                | `uint8_t compat_flags`     | 兼容性标志位                         |              | 无实际意义                                                   |
| 4                | `uint8_t seq`              | 序列号                               | 0 - 255      | 用于检测通信的丢失率                                         |
| 5                | `uint8_t sysid`            | System ID (发送方)                   | 1 - 255      | 发送方的System ID                                            |
| 6                | `uint8_t compid`           | Component ID (发送方)                | 1 - 255      | 发送方的 component ID，例如 飞控、相机等，可参考 [MAV_COMPONENT](https://mavlink.io/en/messages/common.html#MAV_COMPONENT) |
| 7 to 9           | `uint32_t msgid:24`        | Message ID (low, middle, high bytes) | 0 - 16777215 | 消息ID，即payload ID                                         |
| 10 to (n+10)     | `uint8_t payload[max 255]` | 数据帧                               |              | 数据                                                         |
| (n+11) to (n+12) | `uint16_t checksum`        | 校验帧 (low byte, high byte)         |              | 校验位 (不包括 `magic`帧)，包括一个额外校验帧[CRC_EXTRA](https://mavlink.io/en/guide/serialization.html#crc_extra) |
| (n+12) to (n+26) | `uint8_t signature[13]`    | 签名帧                               |              | 确保消息安全                                                 |

### MAVLink 2签名机制

MAVLink 2和MAVLink 1最大的区别就在于增加了13bytes的签名帧，签名帧共由link id、timestamp和signature组成。

![MAVLink 2 Signed](https://mavlink.io/assets/packets/packet_mavlink_v2_signing.png)

- **linkID**(8 bits)：link编号，一般等同于channel的编号。
- **timestamp**(48 bits): 时间戳，单位：毫秒。
- **signature**(48 bits): 由签名位前所有的数据通过哈希单向散列算法（SHA-256）计算得到。

**密钥管理**

密钥是一组32bytes的二进制数据，仅由通讯双方掌握，用于签名位的计算。可以通过SETUP_SIGNING消息进行密钥传递。

为了避免密钥泄露，在log管理中，应避免记录SETUP_SIGNING消息。

**签名帧校验规则及接收条件**

签名帧必须同时满足以下条件才可以被接收：

- 时间戳必须大于上一个包
- 必须和计算得到的48位签名对应上
- 若时间戳晚于本地系统1分钟，则不能被接收（超时）

关于签名帧的官方补充资料（下载需要翻墙）：https://docs.google.com/document/d/1ETle6qQRcaNWAmpG2wz0oOpFKSF_bcTmYMQvtTGI8ns/edit?usp=sharing

**签名流程：**

- 首选需要启动签名机制，默认情况下签名机制是不启动。即在`mavlink_finalize_message_chan()`函数中增加如下代码启动签名机制

  ``` c
  //在编码中，启动签名机制
  //声明一个签名帧结构体
  mavlink_signing_t signing;
  memset(&signing, 0, sizeof(signing));
  //读取指定秘钥
  memcpy(signing.secret_key, secret_key_test, 32);
  //link id赋值
  signing.link_id = (uint8_t)chan;
  //时间戳赋值，注意单位是ms
  uint64_t timestamp_now = get_time_msec();
  signing.timestamp = timestamp_now; 
  //标志位设定
  signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
  //这个标志位暂时不会用，可以不设置
  //signing.accept_unsigned_callback = accept_unsigned_callback;
  //将签名帧结构体复制到状态结构体中
  status->signing = &signing;
  ```

- `mavlink_sign_packet()`函数通过传入的签名帧结构体对签名帧进行赋值操作，主要是调用`mavlink_sha256.h`文件中的API函数，并通过SHA-256散列算法进行加密处理。

  ``` c
  //sha256算法加密的过程
  //初始化，设定8个哈希初值
  mavlink_sha256_init(&ctx);
  //加入密钥
  mavlink_sha256_update(&ctx, signing->secret_key, sizeof(signing->secret_key));
  //加入帧头，payload之前的部分
  mavlink_sha256_update(&ctx, header, header_len);
  //加入payload帧
  mavlink_sha256_update(&ctx, packet, packet_len);
  //加入CRC？
  mavlink_sha256_update(&ctx, crc, 2);
  //加入link_id及时间戳
  mavlink_sha256_update(&ctx, signature, 7);
  //生成最终的48位密码，6个byte，并存入了签名帧中
  mavlink_sha256_final_48(&ctx, &signature[7]);
  ```

- 签名帧共48bit，为SHA-256散列算法的前48位。参与SHA-256算法计算的有密钥、帧头、载荷帧、CRC、linkID及时间戳，相关API函数在mavlink_sha256.h中



## 安装及入门

### 安装MAVLink及生成MAVLink库

**安装MAVLink工具链**

此步主要是安装MAVLink工具链，该工具链将用于生成MAVLink库。由于安装步骤较简单，直接按照手册流程安装即可。（支持windows和linux两个平台）

直达链接：https://mavlink.io/en/getting_started/installation.html

若不需要自定义消息，其实可以跳过此步骤，直接下载并使用官方生成好的库进行开发移植即可，官方提供以下两个已经生成好的库

- [c_library_v2](https://github.com/mavlink/c_library_v2) (MAVLink 2)
- [c_library_v1](https://github.com/mavlink/c_library_v1) (MAVLink 1)

**生成MAVLink库**

两种方式，一种为**mavgenerate.py**（图形界面方式），另一种为**mavgen.py**（命令行工具）。生成工具的代码均在mavlink仓库中开源，可以进行二次开发，关于生成工具的源码解析，可以参看如下链接

- https://blog.csdn.net/lipi37/article/details/104599081/

此处以第一种方式为例，介绍MAVLink库的生成步骤

1. 使用官方提供的XML文件或者自定义XML文件

   - XML文件中包含了各种宏变量、消息payload载荷的定义等等。官方提供了几种XML文件的示例，其中，common.xml为PX4中所使用的消息集
   - 注意v1.0和v2.0的XML文件的内容和写法是有区别的，但暂时不清楚具体的区别
   - 关于XML文件的更多介绍请参阅文档中的如下链接
     - https://mavlink.io/en/guide/xml_schema.html
     - https://mavlink.io/en/guide/define_xml_element.html
     - https://mavlink.io/en/messages/

2. 如果环境变量没有配置成功，则需要打开到mavgenerate.py文件所在的目录输入以下指令

   ``` sh
   python3 -m mavgenerate
   ```

   ![image-20200512143410129](/home/qi/.config/Typora/typora-user-images/image-20200512143410129.png)

   第一个**XML**栏需要指定XML文件，官方提供的几个XML文件示例位于~/mavlink/message_definitions/v1.0，第二个**OUT**栏为生成库的地址，**语言**可选择C语言等，协议需要选择MAVLink 1或者MAVLink 2，剩下两个具体意义不是很清楚，但都勾上，然后点击**Generate**即可。

**MAVLink库简单介绍**

C语言的库均为.h头文件，构成为一个与XML同名的文件夹和几个单独的h文件。文件夹中的文件依据XML内容生成，而外部的几个头文件则是不变的。

以common消息集及MAVLink 2为例，介绍下MAVLink库中所包含的文件。生成后，得到一个common文件夹（里面也全是.h文件）和7个单独的.h文件，结构如下

- **common文件夹**
  - **结构图**<img src="/home/qi/.config/Typora/typora-user-images/image-20200507104613585.png" alt="image-20200507104613585" style="zoom:33%;" />
  - `mavlink.h`     总的头文件，使用库时，仅需要引用该头文件
  - `common.h `   该文件与XML文件同名，声明了XML文件中定义的宏变量及消息
  - `mavlink_msg_xxxxxx.h` 
  - ... xxxxx代表消息名称，这些文件中声明了与该消息相关的结构体及解码编码API函数
  - `mavlink_msg_xxxxxx.h`
  - `version.h`    版本信息
  - `testsuite.h `　测试用？不清楚
- `protocol.h` 主h文件
- `mavlink_helpers.h`  定义了收发MAVLink消息所需的API函数
- `checksum.h` 校验工具函数，用于检验位的计算
- `mavlink_sha256.h`  签名工具函数，用于签名帧的计算和检验，MAVLink 1生成的库没有此头文件
- `mavlink_conversions.h` 坐标转换等工具函数，和消息收发无关系
- `mavlink_get_info.h`　查询类工具函数，通过msgid、name等查询消息
- `mavlink_types.h` 定义了MAVLink正常运行所需要的自定义结构体

### 使用MAVLink库

不同编程语言调用MAVLink库可能略有区别，此处仅介绍C语言的用法。

1. 将所得到的文件全部放入include文件夹中，

2. 引用头文件，根据前面的分析，此处仅需引用一个头文件即可，其他h文件都被`mavlink.h`所包含

   ```c
   #include <your_dialect/mavlink.h>
   ```

3. 声明一个**mavlink_system_t**全局变量用于指定本端的`system id`及`component id`

   ```cpp
   mavlink_system_t mavlink_system = {
       1, // System ID (1-255)
       1  // Component ID (a MAV_COMPONENT value)
   };
   ```

### MAVLink消息收发的源码解读

**发送MAVLink消息**

1. **payload帧赋值**。将本地待发送的数据转储至指定结构体，如`mavlink_set_position_target_local_ned_t`，结构体定义在对应头文件中可以查询得到。

   ``` c
   MAVPACKED(
   typedef struct __mavlink_local_position_ned_t {
    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
    float x; /*< [m] X Position*/
    float y; /*< [m] Y Position*/
    float z; /*< [m] Z Position*/
    float vx; /*< [m/s] X Speed*/
    float vy; /*< [m/s] Y Speed*/
    float vz; /*< [m/s] Z Speed*/
   }) mavlink_local_position_ned_t;
   ```

2. **消息编码**。使用上一步得到的结构体和对应的编码API函数生成标准的MAVLink消息结构体`mavlink_message_t`，定义如下

   ``` c
   MAVPACKED(
   typedef struct __mavlink_message {
   	uint16_t checksum;      ///< sent at end of packet
   	uint8_t magic;          ///< protocol magic marker
   	uint8_t len;            ///< Length of payload
   	uint8_t incompat_flags; ///< flags that must be understood
   	uint8_t compat_flags;   ///< flags that can be ignored if not understood
   	uint8_t seq;            ///< Sequence of packet
   	uint8_t sysid;          ///< ID of message sender system/aircraft
   	uint8_t compid;         ///< ID of the message sender component
   	uint32_t msgid:24;      ///< ID of message in payload
   	uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8];
   	uint8_t ck[2];          ///< incoming checksum bytes
   	uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];
   }) mavlink_message_t;
   ```

   **编码流程：**

   编码将调用定义在对应的消息h文件中的`mavlink_msg_xxxxx_encode() `函数， 函数使用方式如下

   ``` c
   mavlink_msg_local_position_ned_encode(system_id, component_id, &message, &local_position);
   ```

   其中，`system_id`和`companion_id`为之前定义的全局变量，`message`为将存入的结构体，`local_position`为待编码的结构体，并嵌套调用如下API函数

   - `mavlink_msg_xxxxx_pack()  ` ，定义在对应的消息h文件中
   - `mavlink_finalize_message() `  ，定义在`mavlink_helpers.h`
   - `mavlink_finalize_message_chan()`，定义在`mavlink_helpers.h`，设定了`MAVLINK_COMM_0`
   - `mavlink_finalize_message_buffer() `，定义在`mavlink_helpers.h`，根据`MAVLINK_COMM_0`设定了`mavlink_status_t`
   - `mavlink_finalize_message_buffer()` 是最终生成`mavlink_message_t`结构体的函数，此函数对传递的消息进行逐位赋值。若启用了签名机制（默认是不启用的，即使是MAVLink 2协议），则调用`mavlink_sign_packet()`函数进行签名帧的赋值。（签名流程见前文）

3. **消息发送**。通过如下API函数将`mavlink_message_t`结构体转化为字符数组，该函数将返回`buf`数组的长度。随后便可以通过端口发送该数据。

   ``` c
   unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
   ```

4. **发送流程结束**。

**接收MAVLink消息**

1. **消息读取**。使用下述指令拷贝读取到的数据至指定结构体`mavlink_message_t`中，这个函数也调用了一系列函数，包括`checksum`位的校验等等，此处不做扩展

   ``` c
   msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
   ```

   - `MAVLINK_COMM_1`: ID of the current channel.
   - `cp`: The char to parse.
   - `mavlink_message_t* message`: On success, the decoded message. NULL if the message couldn't be decoded.
   - `mavlink_status_t* status`: The channel statistics, including information about the current parse state.

2. **签名帧校验**。不设置签名位的话，可跳过此步。需要定义全局变量`signing`和`signing_streams`，且时间戳需要实时更新，因为签名帧校验需要检验当前时间。具体规则见后续签名帧的具体介绍。

   ``` c
   uint64_t timestamp_test = get_time_msec();
   signing.timestamp = timestamp_test; 
   bool check_signature = mavlink_signature_check(&signing, &signing_streams,&message);
   ```

3. **消息解码**。根据`msgid`来调用不同的函数来解码，并存储为指定的结构体。以心跳包`MAVLINK_MSG_ID_HEARTBEAT`为例，其中`message`数据类型为`mavlink_message_t`，`current_messages.heartbeat`的数据类型为`mavlink_heartbeat_t`

   ``` c
   mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
   ```

4. **本地转储**。解码成功后，转存为本地变量。

4. **接收流程结束**。

**备注 2020.5.12**

PX4源码中只使用mavlink提供的部分API函数，以发送为例，只使用了`xxxx_encode()`函数，但是`mavlink_msg_to_send_buffer`并未使用。

以上针对的是mavlink模块中的实际代码，在sitl仿真及gazebo子模块中依然使用的是上述的流程



## Demo源码

大部分流程可参照**c_uart_interface_example**仓库的写法，仓库链接：https://github.com/mavlink/c_uart_interface_example。**c_uart_interface_example**中共写了五个类：mavlink_control类、autopilot_interface类、generic_port类、serial_port类、udp_port类，mavlink_control为最顶层模块，autopilot_interface次顶层，剩下的均为通信底层，共提供了UDP和串口两种方式。

**c_uart_interface_example**程序较为简单，但只支持MAVLink 1协议。后续会提供我开发的支持MAVLink 2协议的demo供参考使用。







###　Routing

Systems/components should process a message locally if any of these conditions hold:

- It is a broadcast message (`target_system` field omitted or zero).
- The `target_system` matches its system id and `target_component` is broadcast (`target_component` omitted or zero).
- The `target_system` matches its system id and has the component's `target_component`
- The `target_system` matches its system id and the component is unknown (i.e. this component has not seen any messages on any link that have the message's `target_system`/`target_component`).

Systems should forward messages to another link if any of these conditions hold:

- It is a broadcast message (`target_system` field omitted or zero).
- The `target_system` does not match the system id *and* the system knows the link of the target system (i.e. it has previously seen a message from `target_system` on the link).
- The `target_system` matches its system id and has a `target_component` field, and the system has seen a message from the `target_system`/`target_component` combination on the link.

### xml文件的规则和编写

## 服务

带交互的双向消息