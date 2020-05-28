# 【代码解读】PX4位置控制

相关的文件

~src/modules/mc_pos_controller  旋翼无人机位置控制模块

~src/lib/FlightTasks 飞行任务库

及其他一些库



- 使用FlightTasks来改变模式，并在不同模式下读取期望值

- 控制算法统一使用PositionControl类，和以前一样为串级控制，但目前支持了速度前馈控制，加速度前馈控制依然不支持
- 起飞会使用单独的控制算法和控制逻辑
- 控制频率取决于local_position的发布频率
- 目前看FlightTasks只和位置控制相关，即逻辑层都在位置控制



相关代码

~src/modules/mc_att_controller  旋翼无人机姿态控制模块



- 主函数入口：MulticopterAttitudeControl::Run()
- 控制频率取决于角速度更新频率，限幅为50Hz 到 5000Hz
- 控制逻辑
  - 先执行角速度环控制（前馈+PID，此处前馈为rate_sp，非加速度项前馈），并发布控制量`actuator_controls_s`（3个力矩+力）（这里没有限制幅度的操作）
  - 若角度控制被激活，则执行角度控制，并计算发布角速度期望值
  - 现在姿态控制与offboard模式切换无关，若在offboard模式下，则位置控制不会被激活，不会发布角度期望值。
  - offboard一样只支持角度和角速度控制



P.S： 同样offboard模式下可以直接发布`set_actuator_control_target`MAVLink消息来给定最终期望值





px4iofirmware 不启用



drivers

- dshot(    DEPENDS

  ​        arch_io_pins

  ​        arch_dshot

  ​        mixer

  ​        mixer_module

  ​        output_limit)

- px4fmu(    DEPENDS

  ​        arch_io_pins

  ​        mixer

  ​        mixer_module

  ​        output_limit

  ​    )

- pwm_out_sim(    DEPENDS

  ​        mixer

  ​        mixer_module

  ​        output_limit)

lib

- mixer
- mixer_module
- output_limit



先从PX4仿真中使用的`pwm_out_sim`模块开始分析（仿真不涉及底层io驱动，较容易理解）

- 声明一个MixingOutput类，该类位于`mixer_module`中

  ``` c++
  MixingOutput _mixing_output{MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
  ```

- 在主循环中循环执行以下函数，该函数的功能如下

  ``` c++
  _mixing_output.update();
  ```

  - 订阅`actuator_controls`

    ``` c++
    _control_subs[i].copy(&_controls[i])
    ```

  - 启动混控，其中`_mixers`是`MixerGroup`类，定义在lib/mixer模块，输出outputs是一组[-1,1]的值

    ``` c++
    /* do mixing */
    // 启用混控
    float outputs[MAX_ACTUATORS] {};
    const unsigned mixed_num_outputs = _mixers->mix(outputs, _max_num_outputs);
    ```

  - 限幅，并计算PWM值（上下限在初始化的时候已设置好）

    ``` c++
    /* the output limit call takes care of out of band errors, NaN and constrains */
    output_limit_calc(_throttle_armed, armNoThrottle(), mixed_num_outputs, _reverse_output_mask,
                      _disarmed_value, _min_value, _max_value, outputs, _current_output_value, &_output_limit);
    
    
    //计算PWM值
    effective_output[i] = control_value * (max_output[i] - ramp_min_output) / 2 + (max_output[i] + ramp_min_output) / 2;
    ```

    `min_pwm`和`max_pwm`分别对应系统参数中的PWM_MIN和PWM_MAX（可在QGC参数列表中查询得到，默认值是1000和2000）。如果某个电机计算出来的控制量为 x（x是一个在[-1, 1]区间的量），则计算出来的pwm值为
    $$
    \text{pwm} = 500 \cdot x + 1500
    $$
    
- 发布混控结果，即发布话题`actuator_outputs`
  
  ``` c++
    setAndPublishActuatorOutputs(mixed_num_outputs, actuator_outputs);
    ```
  
- 结束

同理，来分析下实际飞行中的`px4fmu`模块

- 声明一个MixingOutput类，该类位于`mixer_module`中

  ``` c++
  MixingOutput _mixing_output{FMU_MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, true};
  ```

- 在主循环中执行更新函数，同上

  ``` c++
  _mixing_output.update();
  ```

- 执行一些与解锁上锁相关的操作

- 设定pwm频率。若`pwm_rate`设置为0，则为oneshot模式（最高2000hz），一般正常PWM频率是400Hz

  ``` c
  update_current_rate();
  {
      // 更改订阅`actuator_controls`的频率
      _mixing_output.setMaxTopicUpdateRate(update_interval_in_us);
  }
  ```

- 结束循环



dshot模块同样是以上的逻辑，但目前并不清楚如何启动dshot及其中的逻辑有何具体区别



所以，最核心的部分还是混控的计算过程

``` c
const unsigned mixed_num_outputs = _mixers->mix(outputs, _max_num_outputs);
```



1. 脚本读取。对于quad_x机型，读取`~/ROMFS/px4fmu_common/mixers/quad_x.main.mix`文件

   ``` bash
   R: 4x 10000 10000 10000 0
   AUX1 Passthrough
   M: 1
   S: 3 5 10000 10000 0 -10000 10000
   AUX2 Passthrough
   M: 1
   S: 3 6 10000 10000 0 -10000 10000
   ```

2. 上述脚本文件中R代表旋翼无人机。	因此，这里会新定义一个**MultirotorMixer**的类。具体过程不叙述了，总之，混控库中使用的是**MultirotorMixer**类

   ``` c
   int MixerGroup::load_from_buf(const char *buf, unsigned &buflen)
   {
       //...
        case  'R':
      		m =  MultirotorMixer::from_text(_control_cb, _cb_handle, p, resid);
         break;
       //...
   }
   ```

   

这后面基本没怎么变，看之前写的控制分配.md即可



**理论混控计算**

对于x型布局的四旋翼，控制效率模型为
$$
\begin{bmatrix}
   \tau _x\\
   \tau _y \\
   \tau _z \\
   {T}   \\
  \end{bmatrix}= 
  \begin{bmatrix}
   -{\frac{{\sqrt 2 }}{2}d{c_T}} & {\frac{{\sqrt 2 }}{2}d{c_T}} & {\frac{{\sqrt 2 }}{2}d{c_T}} & -{\frac{{\sqrt 2 }}{2}d{c_T}} \\
   {\frac{{\sqrt 2 }}{2}d{c_T}} & -{\frac{{\sqrt 2 }}{2}d{c_T}} & {\frac{{\sqrt 2 }}{2}d{c_T}} & -{\frac{{\sqrt 2 }}{2}d{c_T}} \\
   {c_M} &  {c_M} &  -{c_M} &  -{c_M} \\
   {c_T} &  {c_T}  & {c_T}  &  {c_T}  \\
  \end{bmatrix}
  \begin{bmatrix}
   \varpi _1^2  \\
   \varpi _2^2  \\
   \varpi _3^2  \\
   \varpi _4^2  \\
  \end{bmatrix} = \bf{M}_4
  \begin{bmatrix}
   \varpi _1^2  \\
   \varpi _2^2  \\
   \varpi _3^2  \\
   \varpi _4^2  \\
  \end{bmatrix}
$$
最左边是控制算法计算出来的控制量（3个力矩+1一个机体z轴升力），d为机臂长度，c_t为升力常亮，c_m为阻力常量，最右边为电机的转速。

但是，一般直接建立的是控制量与电机所产生拉力的关系，如下
$$
\begin{bmatrix}
   \tau _x\\
   \tau _y \\
   \tau _z \\
   {T}   \\
  \end{bmatrix}= 
  \begin{bmatrix}
   -{\frac{{\sqrt 2 }}{2}d} & {\frac{{\sqrt 2 }}{2}d} & {\frac{{\sqrt 2 }}{2}d} & -{\frac{{\sqrt 2 }}{2}d} \\
   {\frac{{\sqrt 2 }}{2}d} & -{\frac{{\sqrt 2 }}{2}d} & {\frac{{\sqrt 2 }}{2}d} & -{\frac{{\sqrt 2 }}{2}d} \\
   {\frac{c_M}{c_T}} &  {\frac{c_M}{c_T}} &  -{\frac{c_M}{c_T}} &  -{\frac{c_M}{c_T}} \\
   {1} &  {1}  & {1}  &  {1}  \\
  \end{bmatrix}
  \begin{bmatrix}
   T _1  \\
   T _2  \\
   T _3 \\
   T _4  \\
  \end{bmatrix} =\bf{P}_4
  \begin{bmatrix}
   T _1  \\
   T _2  \\
   T _3 \\
   T _4  \\
  \end{bmatrix}
$$
理想的控制分配过程为：
$$
\begin{bmatrix}   \text{output}[1]\\   \text{output}[2]\\   \text{output}[3]\\  \text{output}[4]  \\  \end{bmatrix}= {\frac{1}{4}}
  \begin{bmatrix}
   -{\frac{{\sqrt 2}}{d}} & {\frac{{\sqrt 2}}{d}} & {\frac{{c_T}}{c_M}} & 1 \\
  {\frac{{\sqrt 2}}{d}} & -{\frac{{\sqrt 2}}{d}}  & {\frac{{c_T}}{c_M}} & 1 \\
 {\frac{{\sqrt 2}}{d}} & {\frac{{\sqrt 2}}{d}} & -{\frac{{c_T}}{c_M}} & 1 \\
   -{\frac{{\sqrt 2}}{d}} & -{\frac{{\sqrt 2}}{d}} & -{\frac{{c_T}}{c_M}} & 1 \\
  \end{bmatrix}
\begin{bmatrix}
   \tau _x\\
   \tau _y \\
   \tau _z \\
   {T}   \\
  \end{bmatrix}
$$
此时output对应为真实的电机推力，再依据推力-油门模型转化为油门，再转为PWM信号

**PX4中的混控计算**

在没有

不出现饱和的时候，PX4的控制效率模型为
$$
\begin{bmatrix}   \tau _x\\   \tau _y \\   \tau _z \\   {T}   \\  \end{bmatrix}=    {\frac{1}{2}}\begin{bmatrix}   -{\frac{{\sqrt 2 }}{2}} & {\frac{{\sqrt 2 }}{2}} & {\frac{{\sqrt 2 }}{2}} & -{\frac{{\sqrt 2 }}{2}} \\   {\frac{{\sqrt 2 }}{2}} & -{\frac{{\sqrt 2 }}{2}} & {\frac{{\sqrt 2 }}{2}} & -{\frac{{\sqrt 2 }}{2}} \\   {\frac{1}{2}} &  {\frac{1}{2}} &  - {\frac{1}{2}} &  - {\frac{1}{2}} \\    {\frac{1}{2}} &   {\frac{1}{2}}  &  {\frac{1}{2}}  &   {\frac{1}{2}}  \\  \end{bmatrix}  \begin{bmatrix}   T _1  \\   T _2  \\   T _3 \\   T _4  \\  \end{bmatrix} =\bf{P}_4  \begin{bmatrix}   T _1  \\   T _2  \\   T _3 \\   T _4  \\  \end{bmatrix}
$$

此时，T为[0,1]的油门值，不代表真实的升力，力矩也被限幅在[-1,1]之间。



根据quad_x矩阵的设定，d=1

PX4中的控制分配对应如下方程
$$
\begin{bmatrix}   \text{output}[1]\\   \text{output}[2]\\   \text{output}[3]\\  \text{output}[4]  \\  \end{bmatrix}=   \begin{bmatrix}   
-{\frac{{\sqrt 2 }}{2}} & {\frac{{\sqrt 2 }}{2}}  & 1& 1 \\   
{\frac{{\sqrt 2 }}{2}} & -{\frac{{\sqrt 2 }}{2}} & 1& 1\\ 
 {\frac{{\sqrt 2 }}{2}} & {\frac{{\sqrt 2 }}{2}}  & -1& 1 \\
 -{\frac{{\sqrt 2 }}{2}} & -{\frac{{\sqrt 2 }}{2}}  & -1& 1 \\
 \end{bmatrix}  
\begin{bmatrix}   \tau _x\\   \tau _y \\ \tau _z\\   {T}   \\  \end{bmatrix}
$$
但具体的控制分配计算过程为

1. 先进行roll,pitch和thrust三个通道的控制分配（yaw的优先级比较低）
   $$
   \begin{bmatrix}   \text{output}[1]\\   \text{output}[2]\\   \text{output}[3]\\  \text{output}[4]  \\  \end{bmatrix}=   \begin{bmatrix}   
   -{\frac{{\sqrt 2 }}{2}} & {\frac{{\sqrt 2 }}{2}}  & 1 \\   {\frac{{\sqrt 2 }}{2}} & -{\frac{{\sqrt 2 }}{2}} & 1\\ 
    {\frac{{\sqrt 2 }}{2}} & {\frac{{\sqrt 2 }}{2}}  & 1 \\
    -{\frac{{\sqrt 2 }}{2}} & -{\frac{{\sqrt 2 }}{2}}  & 1 \\
    \end{bmatrix}  
   \begin{bmatrix}   \tau _x\\   \tau _y \\   {T}   \\  \end{bmatrix}
   $$


2. 对上述结果进行限幅处理，防止出现饱和现象

3. 对yaw通道进行控制分配
   $$
   \begin{bmatrix}   \text{output}[1]\\   \text{output}[2]\\   \text{output}[3]\\  \text{output}[4]  \\  \end{bmatrix}= 
   \begin{bmatrix}   \text{output}[1]\\   \text{output}[2]\\   \text{output}[3]\\  \text{output}[4]  \\  \end{bmatrix}+
   \begin{bmatrix}   
   1 \\  1\\  -1\\ -1\\
    \end{bmatrix}  
     \tau _z
   $$
   

4. 限幅处理，防止出现饱和

5. 缩放处理

   - 若指定了推力系数，则先按照推力系数进行缩放

   - 统一缩放至[idle,1]（一般为[-1,1]），即
     $$
     \text{output}[i] = 2*\text{output}[i] - 1
     $$

   这样，roll，pitch，yaw的控制量在[-1, 1]区间，thrust控制量在[0, 1]区间；output[i]在[-1, 1]区间；这样最后算出的pwm值在[1000, 2000]区间。



举例，对于[0,0,0,0.5]这一组控制量（一般0.5为悬停油门），产生的PWM为[1500,1500,1500,1500]