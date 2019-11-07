## base_local_planner

> 参考链接：https://github.com/ros-planning/navigation

### 概述
这个包使用**Trajectory Rollout and Dynamic Window approaches**来做平面上运动的机器人局部导航，控制器基于给定的路径规划和`costmap`生成速度命令后发送给移动基座。
该包适用于全向移动和非全向移动机器人，机器人轮廓可以表示为凸多边形或者圆。
这个包进行了ROS封装，继承了`BaseLocalPlanner`接口，且可以在启动文件中设置ROS参数。
`base_local_planner`包提供了驱动底座在平面移动的控制器，控制器可以连接路径规划器和机器人基座。

  为了让机器人从起始位置到达目标位置，路径规划器使用地图创建运动轨迹。
向目标移动的路上，路径规划器至少需要在地图上的机器人周围创建一个可以表示成栅格地图的评价函数。

```bash
This value function encodes the costs of traversing through the grid cells.
```

该控制器任务就是用这个评价函数确定发送速度和角度（DWA的父类）（dx，dy，dtheta velocities）给机器人基座。



`Trajectory Rollout and Dynamic Window Approach (DWA)`算法基本理念如下:
- 采样机器人当前的状态。`Discretely sample in the robot's control space (dx,dy,dtheta)`
- 用采样的离散点做前向模拟，基于机器人当前状态，预测如果使用空间采样点的速度运动一段时间可能出现的情况。
- 评价前向模拟的每条轨迹，评价标准包括（接近障碍，接近目标，接近全局路径和速度）。丢弃不合法的轨迹（如可能碰到障碍物的轨迹）。
- 根据打分，选择最优路径，并将其对应速度发送给基座。
- 重复上面步骤。

DWA与Trajectory Rollout的区别主要是在**机器人的控制空间采样差异**。`Trajectory Rollout`采样点来源于整个前向模拟阶段所有可用速度集合，而DWA采样点仅仅来源于一个模拟步骤中的可用速度集合。这意味着相比之下DWA是一种更加有效算法，因为其使用了更小采样空间；然而对于低加速度的机器人来说可能Trajectory Rollout更好，因为DWA不能对常加速度做前向模拟。

在实践中，我们经过多次实验发现2种方法性能基本相当，这样的话我们推荐使用效率更高的DWA算法。
一些有用的参考链接:

`在LAGR机器人上使用的Trajectory Rollout算法的讨论`：

> Brian P. Gerkey and Kurt Konolige. "Planning and Control in
> Unstructured Terrain ".

> The Dynamic Window Approach to local control： D. Fox, W. Burgard, and
> S. Thrun. "The dynamic window approach to collision avoidance".

> An previous system that takes a similar approach to control：Alonzo
> Kelly. "An Intelligent Predictive Controller for Autonomous Vehicles".

#### ( 1 ) Map Grid
- 为了有效地评价轨迹，使用了地图栅格。
- 每个控制周期，都会在机器人周围创建栅格（大小为局部costmap）区域，并且全局路径会被映射（相关函数）到这个区域上。 这意味着有一定栅格将被标记为到路径点距离为0，到目标距离为0。接着利用传播算法可以将剩下的标记，记录它们到各自最近的标记为0的点的距离。
- 然后，利用地图栅格进行轨迹评价。

全局目标一般不会出现在地图栅格标记的小区域内，所以为接近目标进行轨迹评价时，这个目标应该是个局部目标，这意味着该小区域内第一个路径点一定是该区域外还有其连续点的那个点。 该小区域的尺寸大小由move_base确定。

##### 震荡抑制
当在`x, y, or theta`维度出现震荡后, 正负值会连续出现。因此，为了抑制震荡影响，当机器人在某方向移动时，对下一个周期的与其相反方向标记为无效，**直到机器人从标记震荡的位置处离开一定距离**。
##### 通用局部路径规划
从ROS groovy版本开始有了一种新的局部路径规划包（DWA）。 这种实现更模块化，其可复用的代码使客制化一种局部路径规划器时候更加容易。 `base_local_planner`基础代码已经被扩展，增加了几个头文件和新的类。

局部规划器的工作原则是在每一个控制周期搜索最佳的局部路径规划。首先局部规划器会生成一些候选轨迹。
其次在检测生成的轨迹是否会与障碍物碰撞。如果没有，规划器就会评价且比较选择出最好的轨迹。
很显然，由于机器人外形（和制动器）以及应用领域的差异，这种工作原则的实例化会不尽相同。

以下的类和接口遵照通用局部路径规划器工作原则允许进行不同实例化。可**以以dwa_local_planner为模板加入自己的代价函数或者轨迹生成器，来创建自定义的局部路径规划器。**

##### ( 1 ) TrajectorySampleGenerator
该接口描述了一种可以生成很多轨迹发生器，每调用一次nextTrajectory()就会返回一个新的轨迹。
SimpleTrajectoryGenerator类可以使用trajectory rollout或DWA原理来生成概述中描述的轨迹。
##### ( 2 ) TrajectoryCostFunction
这个接口包含了最重要的函数`scoreTrajectory(Trajectory &traj)`, 该函数输入轨迹后会输出轨迹评价分数。
如果输出`负分数意味着轨迹无效`；输出分数为正值，对于cost函数来说值越小越好。
每个cost函数有一个比例因子，与其它cost函数比较时候，通过调节比例因子，cost函数影响可以被改变。
base_local_planner包附带了一些在PR2上使用过的cost函数，如下所述。
##### ( 3 ) SimpleScoredSamplingPlanner
这是轨迹搜索的一种简单实现，利用了`TrajectorySampleGenerator`产生的轨迹和一系列`TrajectoryCostFunction`。 它会一直调用`nextTrajectory`()直到发生器停止生成轨迹。对于每一个生成的轨迹，将会把列表中的cost函数都循环调用，并把cost函数返回的正值，负值丢弃。

利用cost函数的比例因子， 最佳轨迹就是cost函数加权求和后最好的的那条轨迹。
##### ( 4 ) Helper classes
###### ( 4.1 ) LocalPlannerUtil()管理器
该帮助接口提供了通用的接口可供所有规划器使用。它管理当前的全局规划，当前的运动约束，以及当前的cost地图（感知障碍物的局部cost地图）。
###### ( 4.2 ) OdometryHelperRos
该类为机器人提供odometry信息。
###### ( 4.3 ) LatchedStopRotateController
理想情况下，局部路径规划器可以让机器人准确停到它应该停止地方。然而在现实中，由于传感器噪声和执行器的不稳定性，机器人会接近到达目标，但其会继续移动，这不是我们想要的结果。

> Ideally a local planner will make a robot stop exactly where it
> should. In practice however, due to sensor noise and actuator
> uncertainty, it may happen that the robot approaches the target spot
> but moves on. This can lead to undesired robot behavior of oscilatting
> on a spot.

`LatchedStopRotateController`是一个不错的控制器，当机器人足够靠近目标时可以迅速启用。
然后，**控制器将执行完全停止和原地旋转朝向目标方向的操作，无论在停止后的机器人位置是否超出目标位置公差范围。**

> The LatchedStopRotateController is a Controller that can be used as
> soon as the robot is close enough to the goal. The Controller will
> then just perform a full stop and a rotation on the spot towards the
> goal orientation, regardless of whether the robot position after the
> full stop leads the robot outside the goal position tolerance.

###### ( 5 ) Cost Functions
###### ( 5.1 ) ObstacleCostFunction
该cost函数类基于感知到的障碍物评价轨迹。如果轨迹经过障碍物则返回负值，其它返回0。
###### ( 5.2 ) MapGridCostFunction
该cost函数类基于轨迹与全局路径或者目标点的接近程度来评价轨迹。它尝试对所有轨迹使用相同的到某个目标或者路径距离的预计算图来优化计算速度。

在dwa_local_planner中, 因目的不同（为让轨迹尽可能接近全局路径，为让机器人朝着局部目标前进，还有为让机器人的头保持指向局部目标），该cost函数具体实现也会不尽相同。因该cost函数使用了试探机制，因此如果使用了不合适的参数可能给出的结果很差甚至根本不能工作。
###### ( 5.3 ) OscillationCostFunction
该cost函数类用以减少一定程度的震荡。虽然该cost函数能有效防止震荡，但是如果使用不合适的参数也可能会得不到一些好的解决方案。
###### ( 5.4 ) PreferForwardCostFunction
该cost函数类适用于类似PR2那种在机器人前方有很好传感器（如tilting laser）布置的机器人。
该cost函数鼓励前向运动，惩罚后向或者其它周围方向性运动。但是这种特性在某些领域机器人可能并不是所期望的，所以仅适合于特定应用的机器人。
#### TrajectoryPlannerROS
`base_local_planner::TrajectoryPlannerROS`是对`base_local_planner::TrajectoryPlanner`的ROS封装。
它在初始化时确定的ROS命名空间内运行，该接口继承了`nav_core`包的`nav_core::BaseLocalPlanner`接口。
如下是`base_local_planner::TrajectoryPlannerROS`的一个应用案例:

```cpp
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>

...

tf::TransformListener tf(ros::Duration(10));
costmap_2d::Costmap2DROS costmap("my_costmap", tf);

base_local_planner::TrajectoryPlannerROS tp;
tp.initialize("my_trajectory_planner", &tf, &costmap);
```

#####  1 ) API Stability

##### ( 1.1 ) Published Topics
~`<name>/global_plan ([nav_msgs/Path])` 
表示的是局部路径规划器目前正在跟随的全局路径规划中的一部分，主要用于可视化。
`<name>/local_plan ([nav_msgs/Pat])` 
表示的是上一个周期局部规划或者轨迹得分最高者，主要用于可视化。

`<name>/cost_cloud ([sensor_msgs/PointCloud2][8]) 
`
用于表示规划的cost栅格，也是用于可视化目的。参数publish_cost_grid_pc用来使用或者关闭该可视化。==New in navigation 1.4.0==
##### ( 1.2 ) Subscribed Topics
`odom ([nav_msgs/Odometry]`

该Odometry信息用于向局部规划器提供当前机器人的速度。
这个里程消息中的速度信息被假定使用的坐标系与TrajectoryPlannerROS对象内cost地图的
robot_base_frame参数指定的坐标系相同。
有关该参数详细介绍请参照 costmap_2d 包。
##### ( 2 ) Parameters
有许多ROS参数可以用来自定义base_local_planner::TrajectoryPlannerROS的行为。（说明这个函数对机器人最后运动影响比较大喽）
这些参数分为几类：`机器人配置，目标公差，前向仿真，轨迹评分，防振和全局计划。`
##### ( 2.1 ) Robot Configuration Parameters

~<name>/`acc_lim_x (double, default: 2.5)` 

机器人在x方向的最大加速度，单位meters/sec^2 。

~<name>/`acc_lim_y (double, default: 2.5)`

机器人在y方向的最大加速度，单位meters/sec^2 。

~<name>/`acc_lim_theta (double, default: 3.2)` 

机器人的最大角加速度，单位radians/sec^2 。

~<name>/max_vel_x (double, default: 0.5)

基座允许的最大线速度，单位meters/sec 。

~<name>/`min_vel_x (double, default: 0.1)` 

基座允许的最小线速度，单位meters/sec 。

设置的最小速度需要保证基座能够克服摩擦。

~<name>`/max_vel_theta (double, default: 1.0)` 

基座允许的最大角速度，单位 radians/sec 。

~<name>`/min_vel_theta (double, default: -1.0)` 

基座允许的最小角速度，单位 radians/sec 。

~<name>/`min_in_place_vel_theta (double, default: 0.4)` 

原地旋转时，基座允许的最小角速度，单位 radians/sec 。

~<name>`/backup_vel (double, default: -0.1)` 

> DEPRECATED (use escape_vel):Speed used for backing up during escapes
> in meters/sec. Note that it must be negative in order for the robot to
> actually reverse. A positive speed will cause the robot to move
> forward while attempting to escape.

`~<name>/escape_vel (double, default: -0.1) `

表示机器人的逃离速度，即背向相反方向行走速度，单位为 meters/sec 。
该值必需设为负值，但若设置为正值，机器人会在执行逃离操作时向前移动。

`~<name>/holonomic_robot (bool, default: true) `

确定是否为全向轮或非全向轮机器人生成速度指令。
对于全向轮机器人，可以向基座发出施加速度命令。 对于非全向轮机器人，不会发出速度指令。
以下参数仅在holonomic_robot设置为true时使用：

`~<name>/y_vels (list, default: [-0.3, -0.1, 0.1, 0.3]) `

The strafing velocities that a holonomic robot will consider in meters/sec
##### ( 2.2 ) 目标公差参数（Goal Tolerance Parameters）

`~<name>/yaw_goal_tolerance (double, default: 0.05) `

The tolerance in radians for the controller in yaw/rotation when achieving its goal差几米停止

` ~<name>/xy_goal_tolerance (double, default: 0.10) `

The tolerance in meters for the controller in the x & y distance when achieving a goal

`~<name>/latch_xy_goal_tolerance (bool, default: false) `

If goal tolerance is latched, if the robot ever reaches the goal xy location it will simply rotate in place, even if it ends up outside the goal tolerance while it is doing so. - New in navigation 1.3.1
###### ( 2.3 ) 前向仿真参数（Forward Simulation Parameters）

~<name>/sim_time (double, default: 1.0) 

前向模拟轨迹的时间，单位为 seconds 。
The amount of time to forward-simulate trajectories in seconds.

`~<name>/sim_granularity (double, default: 0.025)`

在给定轨迹上的点之间的步长，单位为 meters 。
The step size, in meters, to take between points on a given trajectory

`~<name>/angular_sim_granularity (double, default: ~<name>/sim_granularity)`

给定角度轨迹的弧长，单位为 radians 。
The step size, in radians, to take between angular samples on a given trajectory.  ==New in navigation 1.3.1==

`~<name>/vx_samples (integer, default: 3) `

x方向速度的样本数。

The number of samples to use when exploring the x velocity space .

~`<name>/vtheta_samples (integer, default: 20)`

角速度的样本数。

The number of samples to use when exploring the theta velocity space

~`<name>/controller_frequency (double, default: 20.0)` 

调用控制器的频率，单位为 Hz 。

Uses searchParam to read the parameter from parent namespaces if not set in the namespace of the controller. For use with move_base, this means that you only need to set its "controller_frequency" parameter and can safely leave this one unset.
##### ( 2.4 ) 轨迹评分参数（Trajectory Scoring Parameters）

```cpp
cost = pdist_scale * (distance to path from the endpoint of the trajectory in map cells or meters depending on the meter_scoring parameter) 
  + gdist_scale * (distance to local goal from the endpoint of the trajectory in map cells or meters depending on the meter_scoring parameter) 
  + occdist_scale * (maximum obstacle cost along the trajectory in obstacle cost (0-254))
```

该cost函数使用如上公式给每条轨迹评分

~`<name>/meter_scoring (bool, default:false)`

默认false情况下，用单元格作为打分的单位，反之，则用米。

Whether the gdist_scale and pdist_scale parameters should assume that goal_distance and path_distance are expressed in units of meters or cells. Cells are assumed by default.

~`<name>/pdist_scale (double, default: 0.6)`

The weighting for how much the controller should stay close to the path it was given.

~`<name>/gdist_scale (double, default: 0.8)`

The weighting for how much the controller should attempt to reach its local goal, also controls speed.

~`<name>/occdist_scale (double, default: 0.01)` 

The weighting for how much the controller should attempt to avoid obstacles .

~`<name>/heading_lookahead (double, default: 0.325)` 

How far to look ahead in meters when scoring different in-place-rotation trajectories .

~<name>`/heading_scoring (bool, default: false)` 

是否根据机器人前进路径的距离进行评分。
Whether to score based on the robot's heading to the path or its distance from the path.

~`<name>/heading_scoring_timestep (double, default: 0.8)` 

How far to look ahead in time in seconds along the simulated trajectory when using heading scoring .

~<name>`/dwa (bool, default: true)` 

是否使用Dynamic Window Approach (DWA)，或者是否使用Trajectory Rollout。
Whether to use the Dynamic Window Approach (DWA)_ or whether to use Trajectory Rollout (NOTE: In our experience DWA worked as well as Trajectory Rollout and is computationally less expensive. It is possible that robots with extremely poor acceleration limits could gain from running Trajectory Rollout, but we recommend trying DWA first.)


`/publish_cost_grid_pc (bool, default: false) `


Whether or not to publish the cost grid that the planner will use when planning. When true, a sensor_msgs/PointCloud2 will be available on the ~/cost_cloud topic. Each point cloud represents the cost grid and has a field for each individual scoring function component as well as the overall cost for each cell, taking the scoring parameters into account. New in navigation 1.4.0
~<name>/global_frame_id (string, default: odom) 

设置cost_cloud工作坐标系，应该与局部cost地图的global坐标系一致。New in navigation 1.4.0
##### ( 2.5 ) Oscillation Prevention Parameters

~<`name>/oscillation_reset_dist (double, default: 0.05)` 

机器人必须运动多少米远后才能复位震荡标记。
##### ( 2.6 ) Global Plan Parameters

~`<name>/prune_plan (bool, default: true)` 

定义当机器人是否边沿着路径移动时，边抹去已经走过的路径规划。 设置为true时，表示当机器人移动1米后，将1米之前的global路径点一个一个清除。（包括全局的global path和局部的global path）
Defines whether or not to eat up the plan as the robot moves along the path. If set to true, points will fall off the end of the plan once the robot moves 1 meter past them.
#### TrajectoryPlanner

`base_local_planner::TrajectoryPlanner`实现了`DWA and Trajectory Rollout`算法。
如果要在ROS中想使用`base_local_planner::TrajectoryPlanner`, 请使用`TrajectoryPlannerROS wrapper`。
###### ( 1 ) API Stability
虽然C++ API是稳定的。 但是仍然推荐使用TrajectoryPlannerROS wrapper 而不是base_local_planner::TrajectoryPlanner。

参考

```cpp
http://wiki.ros.org/base_local_planner
http://blog.csdn.net/x_r_su/article/details/53380545
http://blog.sina.com.cn/s/blog_135e04d0a0102zpl1.html#cmt_3076623
```
