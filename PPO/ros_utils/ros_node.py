#!/bin/python3

import subprocess
import time
import os
import rospkg
import rospy

"""
与 ROS 节点处理相关的函数。
"""

def ros_node_from_pkg(
    pkg_name,
    node_name,
    launch_master=False,
    launch_master_term=True,
    launch_new_term=True,
    name=None,
    ns="/",
    output="log",
    args=None,  # 新增参数
    port=11311  # 新增参数
) -> bool:
    """
    从包中启动 ROS 节点的函数。

    :param pkg_name: 要从中启动节点的包的名称。
    :type pkg_name: str

    :param node_name: 要启动的节点的名称。
    :type node_name: str

    :param launch_master: 如果 ROSMASTER 没有运行，则启动它。
    :type launch_master: bool

    :param launch_master_term: 如果启动 ROSMASTER，则在外部终端中启动。
    :type launch_master_term: bool

    :param launch_new_term: 是否在新终端（Xterm）中启动进程。
    :type launch_new_term: bool

    :param name: 要启动的节点的名称。
    :type name: str

    :param ns: 要启动的节点的命名空间。
    :type ns: str

    :param output: 输出目标，可以是 log、screen 或 None。
    :type output: str

    :param args: 要传递给节点的额外参数。
    :type args: str

    :return: 如果节点已启动，则返回 True，否则返回 False。
    :rtype: bool
    """
    
    os.environ['ROS_MASTER_URI'] = f'http://localhost:{port}'
    os.environ['GAZEBO_MASTER_URI'] = f'http://localhost:{port + 10}'
    
    rospack = rospkg.RosPack()
    try:
        rospack.get_path(pkg_name)
        rospy.logdebug("找到包...")
    except rospkg.common.ResourceNotFound:
        rospy.logerr("未找到包")
        return False

    if launch_master:
        print("启动 Master")
        try:
            rospy.get_master().getPid()
        except ConnectionRefusedError:
            print("Master 未运行")
            if launch_master_term:
                subprocess.Popen("xterm -e 'roscore' ", shell=True)
            else:
                subprocess.Popen("roscore", shell=True)
            time.sleep(0.1)
        else:
            print("Master 正在运行")

    try:
        rospy.get_master().getPid()
    except ConnectionRefusedError:
        print("Master 未运行")
        return False
    else:
        print("Master 正在运行")

    term_command = "rosrun " + pkg_name + " " + node_name

    if name is not None:
        term_command += " __name:=" + str(name)

    term_command += " __ns:=" + str(ns)
    term_command += " __log:=" + str(output)

    if args is not None:
        term_command += " " + args  # 将额外的参数添加到命令中

    # 设置环境变量
    env_setup = f"export ROS_MASTER_URI=http://localhost:{port}; export GAZEBO_MASTER_URI=http://localhost:{port + 10};"
    term_command = env_setup + term_command

    if launch_new_term:
        term_command = "xterm -e ' " + term_command + "'"

    subprocess.Popen(term_command, shell=True)
    time.sleep(0.1)

    return True


def ros_kill_node(node_name) -> bool:
    """
    杀死 ROS 节点的函数。

    :param node_name: 要杀死的节点的名称。
    :type node_name: str

    :return: 如果节点被杀死，则返回 True，否则返回 False。
    :rtype: bool
    """

    term_command = "rosnode kill " + node_name
    subprocess.Popen("xterm -e ' " + term_command + "'", shell=True).wait()
    return True


def ros_kill_all_nodes() -> bool:
    """
    杀死所有正在运行的 ROS 节点的函数。

    :return: 如果所有节点被杀死，则返回 True，否则返回 False。
    :rtype: bool
    """

    term_command = "rosnode kill -a"
    subprocess.Popen("xterm -e ' " + term_command + "'", shell=True).wait()
    return True


def ros_kill_master() -> bool:
    """
    杀死 ROS master 的函数。

    :return: 如果 master 被杀死，则返回 True，否则返回 False。
    :rtype: bool
    """

    try:
        rospy.get_master().getPid()
    except ConnectionRefusedError:
        print("Master 未运行")
        return True
    else:
        print("Master 正在运行")
        term_command = "rosnode kill -a"
        subprocess.Popen("xterm -e ' " + term_command + "'", shell=True).wait()
        time.sleep(0.1)
        term_command = "killall -9 rosout roslaunch rosmaster nodelet"
        subprocess.Popen("xterm -e ' " + term_command + "'", shell=True).wait()

        return True


def ros_kill_all_processes() -> bool:
    """
    函数用于杀死所有正在运行的与ROS相关的进程。

    :return: 如果所有进程都被杀死，则返回True，否则返回False。
    :rtype: bool
    """

    # 定义要终止的进程命令
    term_command = "killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient xterm rviz"
    # 使用subprocess.Popen执行命令，并等待命令执行完成
    subprocess.Popen("xterm -e ' " + term_command + "'", shell=True).wait()
    # 返回True表示所有进程已被杀死
    return True