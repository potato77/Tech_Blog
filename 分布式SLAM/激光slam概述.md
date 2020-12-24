### 激光SLAM的pipeline

- 激光雷达去畸变
- 激光帧间（核心算法）|前端匹配
- 激光回环检测
- 非线性最小二乘优化（后端优化）

##### 数据处理

- 激光雷达运动畸变去除
- 里程计数据矫正

##### 帧间匹配算法

- ICP（Iterrative Closest Point）
- PI-ICP(point to line Iterrative Closest Point)
- NDT(Normal Distribution Transformation)
- CSM(Correlation Scan Match)

##### 回环检测

- Scan -to-Scan
- Scan-to-Map
- Map-to-Map

##### 后端优化

- 高斯牛顿方法
- LM方法

### 2D激光SLAM的发展

##### Filter-based

- EKF-SLAM
- FastSLAM
- Gmapping
- Optimal RBpf

##### Graph-based

- Globally Consistent Range Scan For Environment Mapping----97
- Incremental Mapping of Large Cyclic Environments----99
- Karto SLAM----10
- Cartographer----16

##### 数据的预处理--非常重要

- 轮式里程计的标定
- 激光雷达运动畸变去除
- 不同系统之间的时间同步

##### 实际环境中的问题

动态物体
 环境变化
 几何结构相似环境
 建图的操作复杂
 全局定位
 地面材质的变化
 地面凹凸不平
 机器人载重的改变

##### 视觉提供的信息

- 高精度的里程信息
- 信息量丰富的视觉地图

##### 融合解决的问题

全局定位
 几何结构相似环境等



作者：徐凯_xp
链接：https://www.jianshu.com/p/5786d022d7c7
来源：简书
著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。