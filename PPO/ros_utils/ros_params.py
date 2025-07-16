#!/bin/python3

import rospy
import rospkg
import os
import rosparam

def ros_load_yaml_from_pkg(pkg_name, file_name, ns='/') -> bool:
    """
    从包中获取 YAML 文件并将其加载到 ROS 参数服务器中。

    :param pkg_name: 包的名称。
    :type  pkg_name: str

    :param file_name: 文件的名称。
    :type  file_name: str

    :param ns: 要加载参数的命名空间。
    :type  ns: str

    :return: 如果文件已加载，则返回 True，否则返回 False。
    :rtype: bool
    """

    rospack = rospkg.RosPack()
    try:
        pkg_path = rospack.get_path(pkg_name)
        rospy.logdebug("找到包...")
    except rospkg.common.ResourceNotFound:
        rospy.logerr("未找到包")
        return False

    file_path = pkg_path + "/config/" + file_name
    if os.path.exists(pkg_path + "/config/" + file_name) is False:
        print("配置文件 " + file_name + " 在 " + file_path + " 中不存在")
        return False

    paramlist = rosparam.load_file(file_path)
    
    for params, namespace in paramlist:
        rosparam.upload_params(ns, params)

    return True

def ros_load_yaml_from_path(file_path, ns='/') -> bool:
    """
    从路径中获取 YAML 文件并将其加载到 ROS 参数服务器中。

    :param file_path: 文件的路径。
    :type  file_path: str

    :param ns: 要加载参数的命名空间。
    :type  ns: str

    :return: 如果文件已加载，则返回 True，否则返回 False。
    :rtype: bool
    """

    if os.path.exists(file_path) is False:
        print("配置文件 " + file_path + " 不存在")
        return False

    paramlist = rosparam.load_file(file_path)
    
    for params, namespace in paramlist:
        rosparam.upload_params(ns, params)

    return True