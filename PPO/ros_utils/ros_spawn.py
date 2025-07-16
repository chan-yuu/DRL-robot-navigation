#!/bin/python3

import rospy
import time

import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
# module_path = os.path.join(current_dir, 'utils', 'ros_utils')
# parent_dir = os.path.dirname(module_path)
sys.path.append(current_dir)

import ros_gazebo
import ros_controllers
import ros_node
import ros_params
import ros_urdf

def init_robot_state_pub(namespace="/", max_pub_freq=None, launch_new_term=False) -> bool:
    """
    初始化机器人状态发布器的函数。

    :param namespace: 机器人的命名空间。
    :type namespace: str

    :param max_pub_freq: 发布器的最大发布频率。
    :type max_pub_freq: float

    :param launch_new_term: 是否在新终端（Xterm）中启动进程。
    :type launch_new_term: bool

    :return: 如果发布器已初始化，则返回 True。
    :rtype: bool
    """

    if max_pub_freq is not None:
        if namespace != "/":
            rospy.set_param(namespace + "/rob_st_pub/publish_frequency", max_pub_freq)
        else:
            rospy.set_param("/rob_st_pub/publish_frequency", max_pub_freq)
            
    return ros_node.ros_node_from_pkg("robot_state_publisher", "robot_state_publisher", launch_new_term=launch_new_term, name="rob_st_pub", ns=namespace)


def spawn_model_in_gazebo(  pkg_name, model_urdf_file, 
                            controllers_file, controllers_list=[],
                            model_urdf_folder="/urdf", ns="/", args_xacro=None, max_pub_freq=None, rob_st_term=False,
                            gazebo_name="robot1", gaz_ref_frame="world", 
                            pos_x=0.0, pos_y=0.0, pos_z=0.0, ori_w=0.0, ori_x=0.0, ori_y=0.0, ori_z=0.0):
    """
    在 Gazebo 中生成模型的函数。

    :param pkg_name: 模型所在的包名。
    :type pkg_name: str

    :param model_urdf_file: 模型的 URDF 文件名。
    :type model_urdf_file: str

    :param controllers_file: 控制器文件的名称。如果为 None，则不会加载任何控制器。
    :type controllers_file: str

    :param controllers_list: 要加载的控制器列表。
    :type controllers_list: list

    :param model_urdf_folder: 模型 URDF 文件所在的文件夹。默认为 "/urdf"。
    :type model_urdf_folder: str

    :param ns: 模型的命名空间。默认为 "/"。
    :type ns: str

    :param args_xacro: 要传递给 xacro 的参数。
    :type args_xacro: list

    :param max_pub_freq: 机器人状态发布器的最大发布频率。
    :type max_pub_freq: float

    :param rob_st_term: 是否在新终端（Xterm）中启动机器人状态发布器。
    :type rob_st_term: bool

    :param gazebo_name: Gazebo 模型的名称。
    :type gazebo_name: str

    :param gaz_ref_frame: Gazebo 模型的参考框架。
    :type gaz_ref_frame: str

    :param pos_x: Gazebo 模型的 X 位置。
    :param pos_y: Gazebo 模型的 Y 位置。
    :param pos_z: Gazebo 模型的 Z 位置。
    :type pos_x: float
    :type pos_y: float
    :type pos_z: float

    :param ori_w: Gazebo 模型的 W 方向。
    :param ori_x: Gazebo 模型的 X 方向。
    :param ori_y: Gazebo 模型的 Y 方向。
    :param ori_z: Gazebo 模型的 Z 方向。
    :type ori_w: float
    :type ori_x: float
    :type ori_y: float
    :type ori_z: float

    :return: 如果模型已生成，则返回 True。
    :rtype: bool
    """

    # 将模型 URDF 加载到参数服务器中
    if ros_urdf.urdf_load_from_pkg(pkg_name, model_urdf_file, "robot_description", folder=model_urdf_folder, ns=ns, args_xacro=args_xacro):
        rospy.logwarn("URDF 文件加载成功")
    else:
        rospy.logwarn("加载 URDF 文件时出错")
        return False
    
    time.sleep(0.1)

    # 初始化机器人状态发布器
    if init_robot_state_pub(namespace=ns, max_pub_freq=max_pub_freq, launch_new_term=rob_st_term):
        rospy.logwarn("机器人状态发布器初始化成功")
    else:
        rospy.logwarn("初始化机器人状态发布器时出错")
        return False

    time.sleep(0.1)

    # 在 Gazebo 中生成模型
    result_spawn, message = ros_gazebo.gazebo_spawn_urdf_param("robot_description", model_name=gazebo_name, robot_namespace=ns, reference_frame=gaz_ref_frame,
                                        pos_x=pos_x, pos_y=pos_y, pos_z=pos_z, ori_w=ori_w, ori_x=ori_x, ori_y=ori_y, ori_z=ori_z,)
    if result_spawn:
        rospy.logwarn("模型生成成功")
    else:
        rospy.logwarn("生成模型时出错")
        rospy.logwarn(message)
        return False

    time.sleep(0.1)

    if controllers_file is not None:
        # 从 YAML 文件中将机器人控制器加载到参数服务器中
        if ros_params.ros_load_yaml_from_pkg(pkg_name, controllers_file, ns=ns):
            rospy.logwarn("机器人控制器加载成功")
        else:
            rospy.logwarn("加载机器人控制器时出错")
            return False

        time.sleep(0.1)

        # 生成控制器
        if ros_controllers.spawn_controllers_srv(controllers_list, ns=ns):
            rospy.logwarn("控制器生成成功")
        else:
            rospy.logwarn("生成控制器时出错")
            return False
    
    return True