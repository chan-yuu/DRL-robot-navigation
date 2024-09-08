仿真环境使用yolo 模型大概训练12-24个小时就到极限了，也没有必要继续去训练了。 推荐的总timestamp是1e6左右

环境安装完成后直接运行： rosrun td3 train_velodyne_td3.py 训练完成后运行： rosrun td3 test_velodyne_td3.py 通过tensorboard来可视化： tensorboard --logdir=runs/
