## <center>How to launch</center>

1.编译通过后，先把rosbag文件播放出来，参考链接：

<https://epan-utbm.github.io/utbm_robocar_dataset/>

![1584684088568](/home/max/.config/Typora/typora-user-images/1584684088568.png)

2.点击how to play下面的launch文件链接，转到github下：

<https://github.com/epan-utbm/utbm_robocar_dataset/tree/baselines/launch>

![1584684370210](/home/max/.config/Typora/typora-user-images/1584684370210.png)

根据README文件，安装GNSS接收驱动，注意各自的ros版本，下面给出我的：

sudo apt-get install ros-kinetic-nmea-navsat-driver

sudo apt-get install ros-kinetic-gps-common

![1584684540428](/home/max/.config/Typora/typora-user-images/1584684540428.png)

![1584684679110](/home/max/.config/Typora/typora-user-images/1584684679110.png)

然后把launch文件下载下来，跟rosbag文件放一起：

![1584685095639](/home/max/.config/Typora/typora-user-images/1584685095639.png)

执行launch文件，将rosbag包，由于我们这里只需要imu和GNSS数据，所以直接用rosbag play:

![1584685419348](/home/max/.config/Typora/typora-user-images/1584685419348.png)

查看下发布了哪些topic：

![1584685455700](/home/max/.config/Typora/typora-user-images/1584685455700.png)

其中/nmea_sentence是GNSS数据，/imu/data是imu数据。

3.在工程中包含launch的文件下目录下运行imu_gps_localization.launch文件，将程序跑起来：

![1584688543854](/home/max/.config/Typora/typora-user-images/1584688543854.png)

运行起来后，再看看有哪些topic:

![1584688596193](/home/max/.config/Typora/typora-user-images/1584688596193.png)

会发现多了些topic，因为我们的lauch文件会运行刚刚安装的GNSS接收驱动nmea_topic_driver：

![1584688760629](/home/max/.config/Typora/typora-user-images/1584688760629.png)

发布两个新的topic：

![1584688837799](/home/max/.config/Typora/typora-user-images/1584688837799.png)

4.关于imu_gps_localization.launch 中imu和GNSS之间的外参，参考utbm_dataset_play.launch中的TF转换部分的内容：

![1585809212281](/home/max/.config/Typora/typora-user-images/1585809212281.png)

上面都是x,y,z,yaw,pitch,roll的表达形式，参考：

![1585809319147](/home/max/.config/Typora/typora-user-images/1585809319147.png)

可以看出，imu和GNSS坐标系完全重合，因此imu_gps_localization中的外参平移部分全部给０。

５.至此程序就可以跑起来了，附上轨迹图：

![1584690063416](/home/max/.config/Typora/typora-user-images/1584690063416.png)