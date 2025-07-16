import math
import os
import random
import subprocess
import time
from os import path

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from squaternion import Quaternion
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import gymnasium as gym
from gymnasium import spaces

# 导入工具函数
from ros_utils.ros_launch import ros_launch_from_pkg

GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.35
TIME_DELTA = 0.1


# Check if the random goal position is located on an obstacle and do not accept it if it is
def check_pos(x, y):
    goal_ok = True

    if -3.8 > x > -6.2 and 6.2 > y > 3.8:
        goal_ok = False

    if -1.3 > x > -2.7 and 4.7 > y > -0.2:
        goal_ok = False

    if -0.3 > x > -4.2 and 2.7 > y > 1.3:
        goal_ok = False

    if -0.8 > x > -4.2 and -2.3 > y > -4.2:
        goal_ok = False

    if -1.3 > x > -3.7 and -0.8 > y > -2.7:
        goal_ok = False

    if 4.2 > x > 0.8 and -1.8 > y > -3.2:
        goal_ok = False

    if 4 > x > 2.5 and 0.7 > y > -3.2:
        goal_ok = False

    if 6.2 > x > 3.8 and -3.3 > y > -4.2:
        goal_ok = False

    if 4.2 > x > 1.3 and 3.7 > y > 1.5:
        goal_ok = False

    if -3.0 > x > -7.2 and 0.5 > y > -1.5:
        goal_ok = False

    if x > 4.5 or x < -4.5 or y > 4.5 or y < -4.5:
        goal_ok = False

    return goal_ok


class GazeboVelodyneEnv(gym.Env):
    """基于原始velodyne_env.py改造的Gymnasium环境，支持多环境训练"""

    def __init__(self, launchfile, environment_dim, max_episode_steps=500, pkg_name="multi_robot_scenario", worker_index=0):
        super(GazeboVelodyneEnv, self).__init__()
        
        self.environment_dim = environment_dim
        self.max_episode_steps = max_episode_steps
        self.current_step = 0
        self.episode_count = 0
        self.pkg_name = pkg_name
        self.launchfile = launchfile
        self.worker_index = worker_index
        
        # 设置环境特定的端口和环境变量
        self.port = 11311 + worker_index * 100
        os.environ['ROS_MASTER_URI'] = f'http://localhost:{self.port}'
        os.environ['GAZEBO_MASTER_URI'] = f'http://localhost:{self.port + 10}'
        
        # 机器人位置信息
        self.odom_x = 0
        self.odom_y = 0
        
        # 目标点设置
        self.goal_x = 1
        self.goal_y = 0.0
        
        # 原始环境的参数
        self.upper = 5.0
        self.lower = -5.0
        self.velodyne_data = np.ones(self.environment_dim) * 10
        self.last_odom = None
        
        # 统计信息
        self.episode_rewards = []
        self.episode_lengths = []
        self.collision_count = 0
        
        # 添加随机种子相关属性
        self._seed = None
        self.np_random = None
        
        # 动作空间：[线速度, 角速度] - 保持原始范围
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]),
            high=np.array([1.0, 1.0]),
            dtype=np.float32
        )
        
        # 观测空间：[激光雷达数据, 距离, 角度差, 线速度, 角速度]
        self.observation_space = spaces.Box(
            low=np.concatenate([
                np.zeros(self.environment_dim),  # 激光雷达数据
                np.array([0.0, -np.pi, -1.0, -1.0])  # 距离, 角度差, 线速度, 角速度
            ]),
            high=np.concatenate([
                np.ones(self.environment_dim) * 30,  # 激光雷达数据
                np.array([np.inf, np.pi, 1.0, 1.0])  # 距离, 角度差, 线速度, 角速度
            ]),
            dtype=np.float32
        )
        
        # 添加render_mode属性
        self.render_mode = None

        self.set_self_state = ModelState()
        self.set_self_state.model_name = "r1"
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 0.0
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0

        # 创建角度分区 - 保持原始逻辑
        self.gaps = [[-np.pi / 2 - 0.03, -np.pi / 2 + np.pi / self.environment_dim]]
        for m in range(self.environment_dim - 1):
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )
        self.gaps[-1][-1] += 0.03

        # 启动仿真环境
        self._launch_simulation()
        # 设置ROS发布者和订阅者
        self._setup_ros_communication()

    def _launch_simulation(self):
        """启动仿真环境"""
        print(f"启动环境 {self.worker_index} 的Gazebo仿真环境 (端口: {self.port})...")
        try:
            # 使用ros_launch_from_pkg启动仿真，每个环境使用不同端口
            args = []  # 多环境时关闭GUI
            if self.port == 11311:
                args = ["gui:=true", "rviz:=true"]
            else:
                args = ["gui:=false", "rviz:=false"]
            time.sleep(self.worker_index * 5)  # 错开启动时间，避免冲突
            success = ros_launch_from_pkg(
                pkg_name=self.pkg_name,
                launch_file=self.launchfile,
                args=args,
                launch_new_term=True,
                port=self.port
            )
            if not success:
                raise RuntimeError(f"启动launch文件失败: {self.pkg_name}/{self.launchfile}")
            
            print(f"环境 {self.worker_index} Gazebo仿真环境启动成功!")
            
        except Exception as e:
            print(f"启动环境 {self.worker_index} 仿真环境失败: {e}")
            raise

    def _setup_ros_communication(self):
        """设置ROS通信"""
        # 在子进程中初始化ROS节点
        try:
            if not rospy.get_node_uri():
                rospy.init_node(f'gazebo_env_{self.worker_index}', anonymous=True)
        except Exception:
            try:
                rospy.init_node(f'gazebo_env_{self.worker_index}', anonymous=True)
            except Exception as e:
                print(f"环境 {self.worker_index} ROS节点初始化失败: {e}")
                raise
        
        # Set up the ROS publishers and subscribers
        self.vel_pub = rospy.Publisher(f"/r1/cmd_vel", Twist, queue_size=1)
        self.set_state = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=10)
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.reset_proxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.publisher = rospy.Publisher(f"goal_point", MarkerArray, queue_size=3)
        self.publisher2 = rospy.Publisher(f"linear_velocity", MarkerArray, queue_size=1)
        self.publisher3 = rospy.Publisher(f"angular_velocity", MarkerArray, queue_size=1)
        
        # Velodyne点云订阅
        self.velodyne = rospy.Subscriber("/velodyne_points", PointCloud2, self.velodyne_callback, queue_size=1)
        self.odom = rospy.Subscriber("/r1/odom", Odometry, self.odom_callback, queue_size=1)

    def seed(self, seed=None):
        """设置环境的随机种子"""
        self._seed = seed
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
            self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def velodyne_callback(self, v):
        """处理Velodyne点云数据的回调函数 - 保持原始逻辑"""
        data = list(pc2.read_points(v, skip_nans=False, field_names=("x", "y", "z")))
        self.velodyne_data = np.ones(self.environment_dim) * 10
        for i in range(len(data)):
            if data[i][2] > -0.2:
                dot = data[i][0] * 1 + data[i][1] * 0
                mag1 = math.sqrt(math.pow(data[i][0], 2) + math.pow(data[i][1], 2))
                mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
                beta = math.acos(dot / (mag1 * mag2)) * np.sign(data[i][1])
                dist = math.sqrt(data[i][0] ** 2 + data[i][1] ** 2 + data[i][2] ** 2)

                for j in range(len(self.gaps)):
                    if self.gaps[j][0] <= beta < self.gaps[j][1]:
                        self.velodyne_data[j] = min(self.velodyne_data[j], dist)
                        break

    def odom_callback(self, od_data):
        self.last_odom = od_data
        
    def step(self, action):
        """执行动作 - 保持原始逻辑"""
        self.current_step += 1
        
        target = False
        truncated = False
        done = False

        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.vel_pub.publish(vel_cmd)
        self.publish_markers(action)

        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        # propagate state for TIME_DELTA seconds
        time.sleep(TIME_DELTA)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

        # read velodyne laser state
        collision_detected, collision, min_laser = self.observe_collision(self.velodyne_data)
        if collision_detected:
            done = True
            self.collision_count += 1
            
        v_state = []
        v_state[:] = self.velodyne_data[:]
        laser_state = [v_state]

        # Calculate robot heading from odometry data - 保持原始逻辑
        self.odom_x = self.last_odom.pose.pose.position.x
        self.odom_y = self.last_odom.pose.pose.position.y
        quaternion = Quaternion(
            self.last_odom.pose.pose.orientation.w,
            self.last_odom.pose.pose.orientation.x,
            self.last_odom.pose.pose.orientation.y,
            self.last_odom.pose.pose.orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

        # Calculate distance to the goal from the robot
        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        # Calculate the relative angle between the robots heading and heading toward the goal
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Detect if the goal has been reached and give a large positive reward
        if distance < GOAL_REACHED_DIST:
            target = True
            done = True

        # 检查episode是否应该截断
        if self.current_step >= self.max_episode_steps:
            truncated = True
            done = True
            
        robot_state = [distance, theta, action[0], action[1]]
        state = np.append(laser_state, robot_state).astype(np.float32)
        
        # 计算奖励 - 保持原始逻辑
        reward = self.get_reward(target, collision, action, min_laser)
        
        info = {
            "target": target, 
            "collision": collision, 
            "min_laser": min_laser,
            "distance": distance,
            "episode_step": self.current_step,
            "collision_rate": self.collision_count / max(1, self.episode_count),
        }
        return state, reward, done, truncated, info

    def reset(self, seed=None, options=None):
        """重置环境 - 保持原始逻辑"""
        super().reset(seed=seed)
        
        if seed is not None:
            self.seed(seed)
        
        self.current_step = 0
        self.episode_count += 1

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service("/gazebo/reset_world")
        try:
            self.reset_proxy()
        except rospy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")

        angle = np.random.uniform(-np.pi, np.pi)
        quaternion = Quaternion.from_euler(0.0, 0.0, angle)
        object_state = self.set_self_state

        x = 0
        y = 0
        position_ok = False
        while not position_ok:
            x = np.random.uniform(-4.5, 4.5)
            y = np.random.uniform(-4.5, 4.5)
            position_ok = check_pos(x, y)
        object_state.pose.position.x = x
        object_state.pose.position.y = y
        object_state.pose.orientation.x = quaternion.x
        object_state.pose.orientation.y = quaternion.y
        object_state.pose.orientation.z = quaternion.z
        object_state.pose.orientation.w = quaternion.w
        self.set_state.publish(object_state)

        self.odom_x = object_state.pose.position.x
        self.odom_y = object_state.pose.position.y

        # set a random goal in empty space in environment
        self.change_goal()
        # randomly scatter boxes in the environment
        # self.random_box()
        self.publish_markers([0.0, 0.0])

        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        time.sleep(TIME_DELTA)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")
            
        v_state = []
        v_state[:] = self.velodyne_data[:]
        laser_state = [v_state]

        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y

        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle

        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        robot_state = [distance, theta, 0.0, 0.0]
        state = np.append(laser_state, robot_state).astype(np.float32)
        
        info = {
            "episode": self.episode_count,
            "collision_rate": self.collision_count / max(1, self.episode_count)
        }
        return state, info

    def change_goal(self):
        """设置新目标 - 保持原始逻辑"""
        # Place a new goal and check if its location is not on one of the obstacles
        if self.upper < 10:
            self.upper += 0.004
        if self.lower > -10:
            self.lower -= 0.004

        goal_ok = False

        while not goal_ok:
            self.goal_x = self.odom_x + random.uniform(self.upper, self.lower)
            self.goal_y = self.odom_y + random.uniform(self.upper, self.lower)
            goal_ok = check_pos(self.goal_x, self.goal_y)

    def random_box(self):
        """随机放置障碍物 - 保持原始逻辑"""
        # Randomly change the location of the boxes in the environment on each reset to randomize the training
        # environment
        for i in range(4):
            name = "cardboard_box_" + str(i)

            x = 0
            y = 0
            box_ok = False
            while not box_ok:
                x = np.random.uniform(-6, 6)
                y = np.random.uniform(-6, 6)
                box_ok = check_pos(x, y)
                distance_to_robot = np.linalg.norm([x - self.odom_x, y - self.odom_y])
                distance_to_goal = np.linalg.norm([x - self.goal_x, y - self.goal_y])
                if distance_to_robot < 1.5 or distance_to_goal < 1.5:
                    box_ok = False
            box_state = ModelState()
            box_state.model_name = name
            box_state.pose.position.x = x
            box_state.pose.position.y = y
            box_state.pose.position.z = 0.0
            box_state.pose.orientation.x = 0.0
            box_state.pose.orientation.y = 0.0
            box_state.pose.orientation.z = 0.0
            box_state.pose.orientation.w = 1.0
            self.set_state.publish(box_state)

    def publish_markers(self, action):
        """发布可视化标记 - 保持原始逻辑"""
        # Publish visual data in Rviz
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0

        markerArray.markers.append(marker)
        self.publisher.publish(markerArray)

        markerArray2 = MarkerArray()
        marker2 = Marker()
        marker2.header.frame_id = "odom"
        marker2.type = marker.CUBE
        marker2.action = marker.ADD
        marker2.scale.x = abs(action[0])
        marker2.scale.y = 0.1
        marker2.scale.z = 0.01
        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.pose.position.x = 5
        marker2.pose.position.y = 0
        marker2.pose.position.z = 0

        markerArray2.markers.append(marker2)
        self.publisher2.publish(markerArray2)

        markerArray3 = MarkerArray()
        marker3 = Marker()
        marker3.header.frame_id = "odom"
        marker3.type = marker.CUBE
        marker3.action = marker.ADD
        marker3.scale.x = abs(action[1])
        marker3.scale.y = 0.1
        marker3.scale.z = 0.01
        marker3.color.a = 1.0
        marker3.color.r = 1.0
        marker3.color.g = 0.0
        marker3.color.b = 0.0
        marker3.pose.orientation.w = 1.0
        marker3.pose.position.x = 5
        marker3.pose.position.y = 0.2
        marker3.pose.position.z = 0

        markerArray3.markers.append(marker3)
        self.publisher3.publish(markerArray3)

    @staticmethod
    def observe_collision(laser_data):
        """碰撞检测 - 保持原始逻辑"""
        # Detect a collision from laser data
        min_laser = min(laser_data)
        if min_laser < COLLISION_DIST:
            return True, True, min_laser
        return False, False, min_laser

    @staticmethod
    def get_reward(target, collision, action, min_laser):
        """奖励函数 - 保持原始逻辑"""
        if target:
            return 100.0
        elif collision:
            return -100.0
        else:
            r3 = lambda x: 1 - x if x < 1 else 0.0
            return action[0] / 2 - abs(action[1]) / 2 - r3(min_laser) / 2

    def close(self):
        """关闭环境"""
        try:
            # 停止所有ROS节点和服务
            if hasattr(self, 'vel_pub'):
                self.vel_pub.unregister()
            if hasattr(self, 'set_state'):
                self.set_state.unregister()
            if hasattr(self, 'publisher'):
                self.publisher.unregister()
            if hasattr(self, 'publisher2'):
                self.publisher2.unregister()
            if hasattr(self, 'publisher3'):
                self.publisher3.unregister()
            if hasattr(self, 'velodyne'):
                self.velodyne.unregister()
            if hasattr(self, 'odom'):
                self.odom.unregister()
        except Exception as e:
            print(f"关闭环境 {self.worker_index} 时出现错误: {e}")