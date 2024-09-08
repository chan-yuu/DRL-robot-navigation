<!--
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-08 15:31:42
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-09-08 15:34:06
 * @FilePath: /aigo_cyun/home/cyun/td3_ws/td3-nav/catkin_ws/td3.md
 * @Description: 
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
-->
仿真环境使用yolo
模型大概训练12-24个小时就到极限了，也没有必要继续去训练了。
推荐的总timestamp是1e6左右

环境安装完成后直接运行：
rosrun td3 train_velodyne_td3.py
训练完成后运行：
rosrun td3 test_velodyne_td3.py
通过tensorboard来可视化：
tensorboard --logdir=runs/