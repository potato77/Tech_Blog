#  【教程】PX4中的控制分配 - Mixer

本文主要讲讲在PX4代码中**pwm计算**的过程，即如何将旋翼姿态控制模块产生的控制指令 ---**三轴力矩+机体轴升力** 转换为对应的**PWM信号**？

如果您想修改**控制分配矩阵**，或者修改**混控算法**，如：设计异型旋翼飞机、在混控中加入电机升力扭矩系数等需求，这篇文章或许对您有一定借鉴意义。

**PX4固件版本**： v1.8.2

**飞行平台**：四旋翼（其他旋翼机类似，但对于固定翼仅有参考意义）

## 前言

一般来说，旋翼无人机的控制分为**位置环控制**+**姿态环控制**两部分：

 - **位置环输入**
     - 设定量：期望位置、期望速度、期望加速度等
     - 状态量：当前位置、当前速度
 - **位置环输出**
      - 期望姿态角（roll + pitch）、期望升力
 - **姿态环输入**
     - 设定量：期望姿态角（roll + pitch + yaw）、期望升力
     - 状态量：当前姿态角、当前角速度
 - **姿态环输出**
      - 期望力矩（Mx、My、Mz）、期望升力（T）

在基于模型的设计中，最后产生的力矩与力的控制量的单位为：N*m和N。但是，在PX4代码（或者说绝大部分飞控代码）中，其姿态环代码的最后输出为**三个[-1, 1]的力矩**及**一个[0, 1]的力**，可以称之为是一个归一化的量吧。之所以代码实现要使用一个归一化的量，是因为每台无人机使用的动力系统（电池+电调+电机+桨）并不一致，故动力系统的各个参数并不统一，导致最后无法进行统一的控制分配。意思也就说：**旋翼姿态控制模块计算出的控制指令值并不等于最后电机产生的实际力和力矩的大小，只是一个归一化的量**。

为什么这也能控制住无人机呢？理论的控制分配矩阵与实际的控制分配矩阵是比例关系的，由于pid算法的鲁棒性，通过参数调整可以这部分比例关系。（详细推导参见全权老师课件）

最近，笔者做了一些姿态环控制算法的设计和修改工作，要想验证基于模型设计的算法可行性，自然是要考虑执行机构的模型，使用正常量纲的力和力矩进行控制分配。因此，除了修改PX4中的内环控制代码，还要修改PX4自带的控制分配代码。

电机参数均由[RC benchmark](https://www.rcbenchmark.com/)的1580电机测试台测得。


## PX4的混控处理

在PX4中，控制分配这部分被称之为mixer（混控）。由于PX4平台特性，混控处理要兼顾各类机型，且还涉及底层信号驱动，整个mixer代码涉及到的文件特别多，本篇文章只是从以下四个方面介绍PX4中的混控处理过程：**pwm计算过程**，**混控相关的代码文件**，**混控矩阵的计算**及**混控的计算过程**。

如果想深入了解PX4中的混控处理流程及细节，可以联系阿木社区购买**混控Mixer学习资料**。

### PX4的混控处理 - pwm计算过程及相关的代码文件

**第一步**

相关文件：`~/src/modules/mc_att_control/mc_att_control_main.cpp`

旋翼姿态控制模块依据控制算法，计算并发布控制指令话题`actuator_controls.msg` ，其中就包含了三个[-1, 1]的力矩及一个[0, 1]的力。

	orb_publish(_actuators_id, _actuators_0_pub, &_actuators);

在日志中对应的是`actuator_controls_0`这个话题，这是因为0号控制组所对应的就是姿态控制组。

**第二步：**

相关文件：`~/src/drivers/px4io/px4io.cpp`

这个文件主要功能是：

 1. **订阅拷贝**姿态环发布的控制指令话题`actuator_controls.msg`，将得到的数据**写入寄存器**PX4IO_PAGE_CONTROLs中。

	    orb_copy(ORB_ID(actuator_controls_0), _t_actuator_controls_0, &controls); 

 2. 从PX4IO_PAGE_SERVOS寄存器中调取计算出的pwm值（计算过程不在此代码），并**发布话题**`actuator_coutputs.msg`。

	     int PX4IO:: io_publish_pwm_outputs();


**为什么这里会有寄存器这个概念？**
底层PX4IO没法直接读取uorb消息，故通过寄存器来与PX4飞控代码通信的。意思就是说，定时器那个模块没法直接读取飞控计算出来的pwm值（uorb消息），需要先存到一个指定的寄存器中，然后它去读取寄存器的值。

这是我猜的，欢迎各位大佬指正。

**第三步**

相关文件：`~/src/modules/px4iofirmware/mixer.cpp`

**以下1与2会在混控计算过程小节中具体讲解。**

 1. 在初始化的时候，读取对应的**脚本**，根据脚本内容，定义一个用于计算混控的mixer类（调用Mixer库）

	    mixer_handle_text_create_mixer();
    
	对于旋翼无人机来说，这里会新定义一个**MultirotorMixer**的类。

	    case  'R':
	          m =  MultirotorMixer::from_text(_control_cb, _cb_handle, p, resid);
	          break;

 2. 进入主循环后，将会调用MultirotorMixer这个类进行**控制分配**的计算，执行代码如下

	    mixed =  mixer_mix_threadsafe(&outputs[0], &r_mixer_limits);

 3. 上一步的控制分配只是相当于计算了每个电机的油门量（对于四旋翼，就是四个[-1, 1]区间的控制量）。然后，计算具体的PWM值，计算出来的pwm值存放于PX4IO_PAGE_SERVOS这个寄存器中。其中调用了pwm_limit这个库，指令如下：

		 pwm_limit_calc(should_arm, should_arm_nothrottle, mixed, r_setup_pwm_reverse, r_page_servo_disarmed,r_page_servo_control_min, r_page_servo_control_max, outputs, r_page_servos, &pwm_limit);

      相关文件：`~/src/lib/pwm_limit/pwm_limit.c`。其中，pwm计算代码 ：

	    effective_pwm[i] = control_value * (max_pwm[i] -  min_pwm[i]) /  2  + (max_pwm[i] +  min_pwm[i]) /  2;
	    
     min_pwm和max_pwm分别对应系统参数中的PWM_MIN和PWM_MAX（可在QGC参数列表中查询得到，默认值是1000和2000）。如果某个电机计算出来的控制量为 x（x是一个在[-1, 1]区间的量），则计算出来的pwm值为 
$$pwm = x*500 + 1500$$

 4. 输出pwm值并驱动电机

		up_pwm_servo_set(i, r_page_servos[i]);

 5. 从这往后就涉及更底层的代码了（控制定时器，发送高低电平之类的吧），我就不太清楚了。


### PX4的混控处理 - 混控矩阵的计算

对于“x”构型的四旋翼来说，在`~/src/lib/mixer/geometries/quad_x.toml`文件中定义了四个电机的位置及转向。
混控矩阵的计算是在代码编译的时候。根据`~/src/lib/mixer/geometries/tools/px_generate_mixers.py`中定义的规则，在编译之后产生的`build`文件夹中可以找到`~/build/nuttx_px4fmu-v5_default/src/lib/mixer/mixer_multirotor_normalized.generated.h`这个文件。文件中自动生成了quad_x这个机型的控制分配矩阵。

    const MultirotorMixer::Rotor _config_quad_x[] = {
    	{ -0.707107,  0.707107,  1.000000,  1.000000 },
    	{  0.707107, -0.707107,  1.000000,  1.000000 },
    	{  0.707107,  0.707107, -1.000000,  1.000000 },
    	{ -0.707107, -0.707107, -1.000000,  1.000000 },};

我们来看看这个矩阵是怎么计算出来的。

 1. 如果在QGC中选择了默认的“x”型四旋翼机架，则在`~/ROMFS/px4fmu_common/init.d/airframes/4001_quad_x`中有如下定义
	
		set MIXER quad_x
	    set PWM_OUT 1234

 3. `quad_x`，即读取~/src/lib/mixer/geometries/quad_x.toml这个文件。
	 
	 懒得贴图了，直接给一个PX4源码[链接](https://github.com/PX4/Firmware/blob/v1.8.2/src/lib/mixer/geometries/quad_x.toml)吧。
 4. 计算混控矩阵，由于原文件是python程序写的，部分定义与常见c语言或matlab不太一致，我整理为matlab代码如下

	    axis      = [0.0, 0.0, -1.0];
        Ct        = 1.0;
        Cm        = 0.05;
        
        position1  = [0.707107, 0.707107, 0.0];
        position2  = [-0.707107, -0.707107, 0.0];
        position3  = [0.707107, -0.707107, 0.0];
        position4  = [-0.707107, 0.707107, 0.0];
        
        torque_matrix(1,:) = Ct * cross(position1, axis) - Cm * [0,0,-1];
        torque_matrix(2,:) = Ct * cross(position2, axis) - Cm * [0,0,-1];
        torque_matrix(3,:) = Ct * cross(position3, axis) - Cm * [0,0,1];
        torque_matrix(4,:) = Ct * cross(position4, axis) - Cm * [0,0,1];
        
        thrust_matrix(1,:) = Ct * axis;
        thrust_matrix(2,:) = Ct * axis;
        thrust_matrix(3,:) = Ct * axis;
        thrust_matrix(4,:) = Ct * axis;
        
        A = [torque_matrix,thrust_matrix]';
  
        B =   pinv(A);
        
        for i = 1:6
            B_norm(i) = norm(B(:,i));
            B_max(i)  = max(abs(B(:,i)));
            B_sum(i)  = sum(B(:,i));
        end
        
        B_norm(1) = max(B_norm(1), B_norm(2)) / sqrt(4 / 2.0);
        B_norm(2) = B_norm(1);
        B_norm(3) = B_max(3);
        B_norm(4) = 1;
        B_norm(5) = B_norm(4);
        B_norm(6) = - B_sum(6) / 4;
        
        B_norm_new = [B_norm(1),B_norm(2),B_norm(3),B_norm(4),B_norm(5),B_norm(6);
                      B_norm(1),B_norm(2),B_norm(3),B_norm(4),B_norm(5),B_norm(6);
                      B_norm(1),B_norm(2),B_norm(3),B_norm(4),B_norm(5),B_norm(6);
                      B_norm(1),B_norm(2),B_norm(3),B_norm(4),B_norm(5),B_norm(6);]
        
        B_px = (B ./ B_norm_new)

 5. 最后B_px这个矩阵就对应最初的_config_quad_x[]这个矩阵。

	       const MultirotorMixer::Rotor _config_quad_x[] = {
	        	{ -0.707107,  0.707107,  1.000000,  1.000000 },
	        	{  0.707107, -0.707107,  1.000000,  1.000000 },
	        	{  0.707107,  0.707107, -1.000000,  1.000000 },
	        	{ -0.707107, -0.707107, -1.000000,  1.000000 },};
	        	
其中，第一排第二项的调用方式为： _rotor[0].pitch_scale。（在MultirotorMixer::mix这个混控函数中你会看到这种调用方式）

**如果是想设计异型旋翼飞机，则可以在第一步的文件中进行修改，比如修改各个电机的位置，或者增加电机数量。**

### PX4的混控处理 - 混控的计算过程

本节对应pwm计算过程中第三步-2的混控计算过程。控制计算调用了mixer这个库，位于`Firmware/src/lib/mixer`中。

 1. 脚本读取。对于quad_x机型，读取`~/ROMFS/px4fmu_common/mixers/quad_x.main.mix`文件。

	    R: 4x 10000 10000 10000 0
	    AUX1 Passthrough
	    M: 1
	    S: 3 5 10000 10000 0 -10000 10000
	    AUX2 Passthrough
	    M: 1
	    S: 3 6 10000 10000 0 -10000 10000

 2. 上述脚本中R代表旋翼无人机。	因此，这里会新定义一个**MultirotorMixer**的类。具体过程不叙述了，总之，混控库中使用的是**MultirotorMixer**类

	    case  'R':
	          m =  MultirotorMixer::from_text(_control_cb, _cb_handle, p, resid);
	          break;
 3. 控制分配计算，落脚于`~/src/lib/mixer/mixer_multirotor.cpp`这个文件中，再具体一点，就是下面这个类函数:

	    unsigned MultirotorMixer::mix(float  *outputs, unsigned  space)

	简单看看它的计算过程：

	**读取控制指令**，从~/px4iofirmware/mixer.cpp的cb_handle中取值，该值就是`actuator_controls.msg`。这里roll_scale，pitch_scale，yaw_scale默认都为1。

		float roll   =  math::constrain(get_control(0, 0) * _roll_scale, -1.0f, 1.0f);
		float pitch  =  math::constrain(get_control(0, 1) * _pitch_scale, -1.0f, 1.0f);
		float yaw    =  math::constrain(get_control(0, 2) * _yaw_scale, -1.0f, 1.0f);
		float thrust =  math::constrain(get_control(0, 3), 0.0f, 1.0f);

	**先是对roll，pitch及thrust进行分配，忽略yaw通道**

	    /* perform initial mix pass yielding unbounded outputs, ignore yaw */
	    for (unsigned i =  0; i < _rotor_count; i++) 
	    {
		    float out = roll *  _rotors[i].roll_scale  +
		    pitch *  _rotors[i].pitch_scale  +
		    thrust *  _rotors[i].thrust_scale;

		    /* calculate min and max output values */
		    if (out < min_out) {  min_out = out;}

		    if (out > max_out) {  max_out = out;}
		    
		    outputs[i] = out;
	    }

	**异常处理**，以下摘自阿木实验室的mixer教程

	> output的范围应该保持在[0, 1]之间，如果有任何一个电机不满足此条件，那么需要通过改变thrust的大小将所有的output平移至[0, 1]之内，平移后如果还不满足，那么就将roll和pitch缩小。如果上下限都不满足，则需要改变thrust，让超出上限及超出下限的值相等。

	再加入yaw进行最后的混控

	    for (unsigned i =  0; i < _rotor_count; i++) 
	    {
			   outputs[i] = (roll *  _rotors[i].roll_scale  +
			   pitch *  _rotors[i].pitch_scale) * roll_pitch_scale +
			   yaw *  _rotors[i].yaw_scale  +
			   (thrust + boost) *  _rotors[i].thrust_scale;
	    }
	我的理解就是防止某个电机出现饱和现象。在不饱和的情况下，这种处理是等同于直接进行roll，pitch，yaw及升力的一步分配的。

	最后，再次对控制量进行一次**缩放**，由[0, 1]缩放至[-1 ,1]
    
	    outputs[i] =  math::constrain(_idle_speed + (outputs[i] * (1.0f  - _idle_speed)), _idle_speed, 1.0f);


**总结一下**，对于普通的”x“型四旋翼，在不饱和的情况下，控制分配计算如下：

$$\begin{bmatrix}
 \text{output 1}\\
 \text{output 2}\\
 \text{output 3} \\
 \text{output 4} \\
 \end{bmatrix}= 
 \begin{bmatrix}
 -{\frac{{\sqrt 2 }}{2}} & {\frac{{\sqrt 2 }}{2}} & 1 & 1 \\
 {\frac{{\sqrt 2 }}{2}} & -{\frac{{\sqrt 2 }}{2}} & 1 & 1 \\
 {\frac{{\sqrt 2 }}{2}} & {\frac{{\sqrt 2 }}{2}} & -1 & 1 \\
 -{\frac{{\sqrt 2 }}{2}} & -{\frac{{\sqrt 2 }}{2}} & -1 & 1 \\
 \end{bmatrix}
 \begin{bmatrix}
\text{  roll } \\
 \text{ pitch  }\\
\text{  yaw  }\\
\text{  thrust  }\\
 \end{bmatrix} $$

 $$\text{output[i]}=   2 *  \text{output[i]} - 1$$
其中，roll，pitch，yaw在[-1, 1]区间，thrust在[0, 1]区间；output[i]在[-1, 1]区间；这样最后算出的pwm值在[1000, 2000]区间。

## SITL仿真中的混控计算

没有使用`~/src/drivers/px4io`及 `~/src/modules/px4iofirmware/mixer.cpp`这里面的代码。而是使用了 `~/src/drivers/pwm_out_sim`，大部分计算过程都是一致的，只是仿真中是不需要最后发送真实的pwm波给执行机构。

大体思路与上述相同，最关键的混控计算步骤都是依赖`~/src/lib/mixer`这个库，对于四旋翼，同样使用**MultirotorMixer**这个类。

所以如果想修改混控，并且在SITL中测试，需要注意你修改的思路应该集中在被调用的那几个文件中。

## 关于作者

阿木实验室科研无人机技术负责人、Mavros培训课程主讲老师、北理工博士

对任何与四旋翼无人机有关的话题感兴趣，欢迎交流。

个人微信号：qyp0210
