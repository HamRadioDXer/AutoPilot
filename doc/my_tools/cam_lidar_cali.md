 ### 使用简单的pnp方法标定世界坐标系下的激光雷达3D Point 和摄像头坐标系下的2D 旋转矩阵和位移

##### 相关工具：
- Numpy
- Matplotlib
- CV2
- ROS（cv_bridge/TF2/ros_numpy/sensor.msgs/message_filters/image_geomotry）
##### 主要思路是：
从激光雷达和相机保存的点云和图片利用matplotlib的gui交互将可视化的角点保存为numpy格式文件，之后利用键盘的handle采集保存多张图片和点云。将保存好的numpy文件导入到slovePnPRansac进行计算，最后在终端打印结果



###### solvePnPRansac相关：



```cpp
/*  max 注释
*   函数功能：用ransac的方式求解PnP问题
*
*   参数：
*   [in]    _opoints                参考点在世界坐标系下的点集；float or double
*   [in]    _ipoints                参考点在相机像平面的坐标；float or double
*   [in]    _cameraMatrix           相机内参
*   [in]    _distCoeffs             相机畸变系数
*   [out]   _rvec                   旋转矩阵
*   [out]   _tvec                   平移向量
*   [in]    useExtrinsicGuess       若果求解PnP使用迭代算法，初始值可以使用猜测的初始值（true），也可以使用解析求解的结果作为初始值（false）。
*   [in]    iterationsCount         Ransac算法的迭代次数，这只是初始值，根据估计外点的概率，可以进一步缩小迭代次数；（此值函数内部是会不断改变的）,所以一开始可以赋一个大的值。
*   [in]    reprojectionErrr        Ransac筛选内点和外点的距离阈值，这个根据估计内点的概率和每个点的均方差（假设误差按照高斯分布）可以计算出此阈值。
*   [in]    confidence              此值与计算采样（迭代）次数有关。此值代表从n个样本中取s个点，N次采样可以使s个点全为内点的概率。
*   [out]   _inliers                返回内点的序列。为矩阵形式
*   [in]    flags                   最小子集的计算模型；
*                                                 SOLVEPNP_ITERATIVE(此方案，最小模型用的EPNP，内点选出之后用了一个迭代)；
*                                                 SOLVE_P3P(P3P只用在最小模型上，内点选出之后用了一个EPNP)      
*                                                 SOLVE_AP3P(AP3P只用在最小模型上，内点选出之后用了一个EPNP)
*                                                 SOLVE_EPnP(最小模型上&内点选出之后都采用了EPNP)
*    返回值：
*         成功返回true，失败返回false；
*/
bool solvePnPRansac(InputArray _opoints, InputArray _ipoints,
	InputArray _cameraMatrix, InputArray _distCoeffs,
	OutputArray _rvec, OutputArray _tvec, bool useExtrinsicGuess,
	int iterationsCount, float reprojectionError, double confidence,
	OutputArray _inliers, int flags)
```



#####  主要功能函数：
按`[ENTER]`启动GUI，并通过选择摄像机和LiDAR框架中棋盘格的四个角点来选择相应的点。选择了16个相应的点用于在棋盘的不同位置和深度进行校准。下面显示了这样一套这样的点。OpenCV的PnP RANSAC方法用于查找相机和LiDAR之间的旋转和平移变换。由于OpenCV的功能在内部对图像进行校正，因此可以从未校正的图像中拾取2D点。

-  读取之前保存的npy格式文件
- 检查Point 2D和Point 3D的shape是否能进行矩阵运算
- 检查角点树是否达到五个点
- 导入相机模型参数
- 开始计算，计算成功打印结果

###### Calibrate the LiDAR and image points using OpenCV PnP RANSAC
###### Requires minimum 5 point correspondences

```shell
Inputs:
    points2D - [numpy array] - (N, 2) array of image points
    points3D - [numpy array] - (N, 3) array of 3D points
```

```powershell
Outputs:
    Extrinsics saved in PKG_PATH/CALIB_PATH/extrinsics.npz
```


```python
def calibrate(points2D=None, points3D=None):
    # Load corresponding points
    folder = os.path.join(PKG_PATH, CALIB_PATH)
    if points2D is None: points2D = np.load(os.path.join(folder, 'img_corners.npy'))
    if points3D is None: points3D = np.load(os.path.join(folder, 'pcl_corners.npy'))
    
    # Check points shape
    #检查角点的shape
    assert(points2D.shape[0] == points3D.shape[0])
    #检查角点的数量【0】代表第一张图像
    if not (points2D.shape[0] >= 5):
        rospy.logwarn('PnP RANSAC Requires minimum 5 points')
        return

    # Obtain camera matrix and distortion coefficients
    camera_matrix = CAMERA_MODEL.intrinsicMatrix()
    dist_coeffs = CAMERA_MODEL.distortionCoeffs()

    # Estimate extrinsics
    success, rotation_vector, translation_vector, _ = solvePnPRansac(points3D,
                                                                     points2D, camera_matrix, dist_coeffs, flags=SOLVEPNP_ITERATIVE)
    if not success: rospy.logwarn('Optimization unsuccessful')

    # Convert rotation vector
    rotation_matrix = Rodrigues(rotation_vector)[0]
    euler = euler_from_matrix(rotation_matrix)
    
    # Save extrinsics
    np.savez(os.path.join(folder, 'extrinsics.npz'),
        euler=euler, R=rotation_matrix, T=translation_vector.T)

    # Display results
    print('Euler angles (RPY):', euler)
    print('Rotation Matrix:', rotation_matrix)
    print('Translation Offsets:', translation_vector.T)
```

##### 采集并保存Point 2D：
- 从ROS读取到的msgs得经过cv_bridge转化为cv2格式
- 利用矫正参数矫正传过来的图像
- matplotlib创建GUI
- 利用event采集角点并显示
- 用save函数保存当前图片和角点文件

###### Runs the image point selection GUI process

```powershell
Inputs:
    img_msg - [sensor_msgs/Image] - ROS sensor image message
    now - [int] - ROS bag time in seconds
    rectify - [bool] - to specify whether to rectify image or not 是否重新矫正

Outputs:
    Picked points saved in PKG_PATH/CALIB_PATH/img_corners.npy 将生成的numpy脚垫保存为相应 numpy格式文件

```

```python
def extract_points_2D(img_msg, now, rectify=False):
    # Log PID
    rospy.loginfo('2D Picker PID: [%d]' % os.getpid())

    # Read image using CV bridge
    try:
        #将ROS CV_bridge转换来的格式转化为cv2格式
        img = CV_BRIDGE.imgmsg_to_cv2(img_msg, 'bgr8')
        #当发生异常是返回日志e
    except CvBridgeError as e: 
        rospy.logerr(e)
        return

    # Rectify image 重新矫正摄像头参数
    if rectify: CAMERA_MODEL.rectifyImage(img, img)
    #
    disp = cvtColor(img.copy(), COLOR_BGR2RGB)

    # Setup matplotlib GUI
    fig = plt.figure()

    ax = fig.add_subplot(111)
    ax.set_title('Select 2D Image Points - %d' % now.secs)
    ax.set_axis_off()
    ax.imshow(disp)

    # Pick points
    picked, corners = [], []
    def onclick(event):
        x = event.xdata
        y = event.ydata
        if (x is None) or (y is None): return

        # Display the picked point
        picked.append((x, y))
        corners.append((x, y))
        rospy.loginfo('IMG: %s', str(picked[-1]))

        if len(picked) > 1:
            # Draw the line
            temp = np.array(picked)
            ax.plot(temp[:, 0], temp[:, 1])
            ax.figure.canvas.draw_idle()

            # Reset list for future pick events
            del picked[0]

    # Display GUI
    fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()

    # Save corner points and image
    rect = '_rect' if rectify else ''
    if len(corners) > 1: del corners[-1] # Remove last duplicate
    save_data(corners, 'img_corners%s.npy' % (rect), CALIB_PATH)
    save_data(img, 'image_color%s-%d.jpg' % (rect, now.secs), 
        os.path.join(CALIB_PATH, 'images'), True)
```

 Point 3D文件的保存：


```css
Runs the LiDAR point selection GUI process

Inputs:
    velodyne - [sensor_msgs/PointCloud2] - ROS velodyne PCL2 message
    now - [int] - ROS bag time in seconds

Outputs:
    Picked points saved in PKG_PATH/CALIB_PATH/pcl_corners.npy
```

```python
def extract_points_3D(velodyne, now):
    # Log PID
    rospy.loginfo('3D Picker PID: [%d]' % os.getpid())

    # Extract points data
    #将雷达点云转换为numpy格式通过ros_numpy
    points = ros_numpy.point_cloud2.pointcloud2_to_array(velodyne)
    #转为list
    points = np.asarray(points.tolist())

    # Select points within chessboard range
    inrange = np.where((points[:, 0] > 0) &
                       (points[:, 0] < 2.5) &
                       (np.abs(points[:, 1]) < 2.5) &
                       (points[:, 2] < 2))
    points = points[inrange[0]]
    print(points.shape)
    if points.shape[0] > 5:
        rospy.loginfo('PCL points available: %d', points.shape[0])
    else:
        rospy.logwarn('Very few PCL points available in range')
        return

    # Color map for the points
    cmap = matplotlib.cm.get_cmap('hsv')
    colors = cmap(points[:, -1] / np.max(points[:, -1]))

    # Setup matplotlib GUI
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('Select 3D LiDAR Points - %d' % now.secs, color='white')
    ax.set_axis_off()
    ax.set_facecolor((0, 0, 0))
    #画散点图
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors, s=2, picker=5)

    # Equalize display aspect ratio for all axes
    max_range = (np.array([points[:, 0].max() - points[:, 0].min(), 
        points[:, 1].max() - points[:, 1].min(),
        points[:, 2].max() - points[:, 2].min()]).max() / 2.0)
    mid_x = (points[:, 0].max() + points[:, 0].min()) * 0.5
    mid_y = (points[:, 1].max() + points[:, 1].min()) * 0.5
    mid_z = (points[:, 2].max() + points[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Pick points
    picked, corners = [], []
    def onpick(event):
        ind = event.ind[0]
        x, y, z = event.artist._offsets3d

        # Ignore if same point selected again
        if picked and (x[ind] == picked[-1][0] and y[ind] == picked[-1][1] and z[ind] == picked[-1][2]):
            return
        
        # Display picked point
        picked.append((x[ind], y[ind], z[ind]))
        corners.append((x[ind], y[ind], z[ind]))
        rospy.loginfo('PCL: %s', str(picked[-1]))

        if len(picked) > 1:
            # Draw the line
            temp = np.array(picked)
            ax.plot(temp[:, 0], temp[:, 1], temp[:, 2])
            ax.figure.canvas.draw_idle()

            # Reset list for future pick events
            del picked[0]

    # Display GUI
    fig.canvas.mpl_connect('pick_event', onpick)
    plt.show()

    # Save corner points
    if len(corners) > 1: del corners[-1] # Remove last duplicate
    save_data(corners, 'pcl_corners.npy', CALIB_PATH)
```

主要监听节点：

```bash
Inputs:
    camera_info - [str] - ROS sensor camera info topic
    image_color - [str] - ROS sensor image topic
    velodyne - [str] - ROS velodyne PCL2 topic
    camera_lidar - [str] - ROS projected points image topic

Outputs: None


```python
def listener(camera_info, image_color, velodyne_points, camera_lidar=None):
    # Start node
    rospy.init_node('calibrate_camera_lidar', anonymous=True)
    rospy.loginfo('Current PID: [%d]' % os.getpid())
    rospy.loginfo('Projection mode: %s' % PROJECT_MODE)
    rospy.loginfo('CameraInfo topic: %s' % camera_info)
    rospy.loginfo('Image topic: %s' % image_color)
    rospy.loginfo('PointCloud2 topic: %s' % velodyne_points)
    rospy.loginfo('Output topic: %s' % camera_lidar)

    # Subscribe to topics
    info_sub = message_filters.Subscriber(camera_info, CameraInfo)
    image_sub = message_filters.Subscriber(image_color, Image)
    velodyne_sub = message_filters.Subscriber(velodyne_points, PointCloud2)

    # Publish output topic
    image_pub = None
    if camera_lidar: image_pub = rospy.Publisher(camera_lidar, Image, queue_size=5)

    # Synchronize the topics by time
    ats = message_filters.ApproximateTimeSynchronizer(
        [image_sub, info_sub, velodyne_sub], queue_size=5, slop=0.1)
    ats.registerCallback(callback, image_pub)

    # Keep python from exiting until this node is stopped
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down')
```







```python
if __name__ == '__main__':

    # Calibration mode, rosrun
    if sys.argv[1] == '--calibrate':
        camera_info = '/sensors/camera/camera_info'
        image_color = '/sensors/camera/image_color'
        velodyne_points = '/sensors/velodyne_points'
        camera_lidar = None
        PROJECT_MODE = False
    # Projection mode, run from launch file
    else:
        camera_info = rospy.get_param('camera_info_topic')
        image_color = rospy.get_param('image_color_topic')
        velodyne_points = rospy.get_param('velodyne_points_topic')
        camera_lidar = rospy.get_param('camera_lidar_topic')
        PROJECT_MODE = bool(rospy.get_param('project_mode'))

    # Start keyboard handler thread
    if not PROJECT_MODE: start_keyboard_handler()

    # Start subscriber
    listener(camera_info, image_color, velodyne_points, camera_lidar)
```

###### 最后结果：






##### 点对应关系保存如下：

```bash
图像点： lidar_camera_calibration/calibration_data/lidar_camera_calibration/img_corners.npy
激光雷达lidar_camera_calibration/calibration_data/lidar_camera_calibration/pcl_corners.npy
```

###### 校准后的外部存储如下：

```bash
lidar_camera_calibration/calibration_data/lidar_camera_calibration/extrinsics.npz
```

```bash
'euler' : Euler Angles (RPY rad)
'R' : Rotation Matrix
'T' : Translation Offsets (XYZ m)
```

###### 矫正后的外参包括：

###### Rotation Matrix

```bash
-9.16347982e-02  -9.95792677e-01  -8.74577923e-05
 1.88123595e-01  -1.72252569e-02  -9.81994299e-01
 9.77861226e-01  -9.00013023e-02   1.88910532e-01
```

###### Euler Angles (RPY rad)

```bash
-0.44460865  -1.35998386   2.0240699
Translation Offsets (XYZ m)
```








