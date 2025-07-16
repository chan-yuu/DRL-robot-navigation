#!/bin/python3

import rospy
import rospkg
import os
import subprocess
import time

def ros_launch_from_pkg(pkg_name, launch_file, args=None, launch_new_term=True, port=11311) -> bool:
    """
    从包中执行 roslaunch 文件的函数。
    :param pkg_name: 包含 launch 文件的包的名称。
    :type pkg_name: str
    
    :param launch_file: launch 文件的名称。
    :type launch_file: str
    
    :param args: 要传递给 launch 文件的参数。
    :type args: str 列表

    :param launch_new_term: 是否在新终端（Xterm）中启动进程。
    :type launch_new_term: bool

    :return: 如果 launch 文件已执行，则返回 True。
    :rtype: bool
    """
    print("port:", port)
    os.environ['ROS_MASTER_URI'] = f'http://localhost:{port}'
    os.environ['GAZEBO_MASTER_URI'] = f'http://localhost:{port + 10}'

    rospack = rospkg.RosPack()
    try:
        pkg_path = rospack.get_path(pkg_name)
        rospy.logdebug("找到包...")
    except rospkg.common.ResourceNotFound:
        rospy.logerr("未找到包")
        return False

    file_path = pkg_path + "/launch/" + launch_file
    if os.path.exists(pkg_path + "/launch/" + launch_file) is False:
        print("launch 文件 " + launch_file + " 在 " + file_path + " 中不存在")
        return False

    term_command = "roslaunch " + pkg_name + " " + launch_file

    if args is not None:
        for arg in args:
            term_command += " " + arg

    # 设置环境变量
    env_setup = f"export ROS_MASTER_URI=http://localhost:{port}; export GAZEBO_MASTER_URI=http://localhost:{port + 10};"
    term_command = env_setup + term_command

    if launch_new_term:
        # term_command = "xterm -e ' " + term_command + "'"
        term_command = f"gnome-terminal -- sh -c '{term_command}'" # ; exec bash
    subprocess.Popen(term_command, shell=True)
    time.sleep(5.0)

    return True

def ros_launch_from_path(launch_file_path, args=None, launch_new_term=True) -> bool:
    """
    从路径中执行 roslaunch 文件的函数。
    :param launch_file_path: launch 文件的路径。
    :type launch_file_path: str

    :param args: 要传递给 launch 文件的参数。
    :type args: str 列表

    :param launch_new_term: 是否在新终端（Xterm）中启动进程。
    :type launch_new_term: bool

    :return: 如果 launch 文件已执行，则返回 True。
    :rtype: bool
    """

    if os.path.exists(launch_file_path) is False:
        print("launch 文件 " + launch_file_path + " 不存在")
        return False

    term_command = "roslaunch " + launch_file_path

    if args is not None:
        for arg in args:
            term_command += " " + arg

    if launch_new_term:
        term_command = "xterm -e ' " + term_command + "'"
    
    subprocess.Popen(term_command, shell=True)
    time.sleep(5.0)

    return True

def ros_kill_launch_process() -> bool:
    """
    杀死所有 roslaunch 进程的函数。

    :return: 如果 roslaunch 进程被杀死，则返回 True。
    :rtype: bool
    """
    term_command = "killall -9 roslaunch"
    subprocess.Popen("xterm -e ' " + term_command + "'", shell=True).wait()
    return True
