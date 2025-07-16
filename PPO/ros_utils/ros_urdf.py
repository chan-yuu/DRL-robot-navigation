#!/bin/python3

import rospy
import rospkg
import os
import xacro

def urdf_load_from_pkg(pkg_name, model_name, param_name, folder="/urdf", ns=None, args_xacro=None, port=11311) -> bool:
    """
    从 ROS 包中加载 URDF 文件到参数服务器。

    :param pkg_name: ROS 包的名称。
    :type pkg_name: str

    :param model_name: 模型文件的名称。
    :type model_name: str

    :param param_name: 参数的名称。
    :type param_name: str

    :param folder: 模型所在的文件夹。默认为 "/urdf"
    :type folder: str

    :param ns: 参数的命名空间。
    :type ns: str

    :param args_xacro: xacro 参数列表，例如：['arg1:=True','arg2:=10.0']
    :type args_xacro: str 列表

    :return: 如果 URDF 文件已加载，则返回 True，否则返回 False。
    :rtype: bool
    """
    
    os.environ['ROS_MASTER_URI'] = f'http://localhost:{port}'
    os.environ['GAZEBO_MASTER_URI'] = f'http://localhost:{port + 10}'

    rospack = rospkg.RosPack()
    try:
        pkg_path = rospack.get_path(pkg_name)
        rospy.logdebug("找到包...")
    except rospkg.common.ResourceNotFound:
        rospy.logerr("未找到包")
        return False

    file_path = pkg_path + folder + "/" + model_name

    if os.path.exists(file_path) is False:
        print("错误：模型路径不存在")
        return False

    list_args = [file_path] 
    if args_xacro is not None:
        list_args = list_args + args_xacro

    opts, input_file_name = xacro.process_args(list_args)
    model = xacro.process_file(input_file_name, **vars(opts))
    encoding = {}
    model_string = model.toprettyxml(indent='  ', **encoding)
    
    if ns is not None and ns != "/":
        final_param_name = ns + "/" + param_name
    else:
        final_param_name = param_name

    rospy.set_param(final_param_name, model_string)
    return True

def urdf_load_from_path(model_path, param_name, ns=None, args_xacro=None) -> bool:
    """
    从文件路径加载 URDF 文件到参数服务器。

    :param model_path: 模型文件的路径。
    :type model_path: str

    :param param_name: 参数的名称。
    :type param_name: str

    :param ns: 参数的命名空间。
    :type ns: str

    :param args_xacro: xacro 参数列表，例如：['arg1:=True','arg2:=10.0']
    :type args_xacro: str 列表

    :return: 如果 URDF 文件已加载，则返回 True，否则返回 False。
    :rtype: bool
    """

    if os.path.exists(model_path) is False:
        print("错误：模型路径不存在")
        return False

    list_args = [model_path] 
    if args_xacro is not None:
        list_args = list_args + args_xacro

    opts, input_file_name = xacro.process_args(list_args)
    model = xacro.process_file(input_file_name, **vars(opts))
    encoding = {}
    model_string = model.toprettyxml(indent='  ', **encoding)

    if ns is not None and ns != "/":
        final_param_name = ns + "/" + param_name
    else:
        final_param_name = param_name

    rospy.set_param(final_param_name, model_string)
    return True

def urdf_parse_from_pkg(pkg_name, model_name, folder="/urdf", args_xacro=None) -> str:
    """
    从 ROS 包中解析 URDF 文件并返回 URDF 字符串。

    :param pkg_name: ROS 包的名称。
    :type pkg_name: str

    :param model_name: 模型文件的名称。
    :type model_name: str

    :param folder: 模型所在的文件夹。默认为 "/urdf"
    :type folder: str

    :param args_xacro: xacro 参数列表，例如：['arg1:=True','arg2:=10.0']
    :type args_xacro: str 列表

    :return: URDF 字符串，如果未找到包或文件，则返回 None。
    :rtype: str
    """
    rospack = rospkg.RosPack()
    try:
        pkg_path = rospack.get_path(pkg_name)
        rospy.logdebug("找到包...")
    except rospkg.common.ResourceNotFound:
        rospy.logerr("未找到包")
        return None

    file_path = pkg_path + folder + "/" + model_name

    if os.path.exists(file_path) is False:
        print("错误：模型路径不存在")
        return None

    list_args = [file_path] 
    if args_xacro is not None:
        list_args = list_args + args_xacro

    opts, input_file_name = xacro.process_args(list_args)
    model = xacro.process_file(input_file_name, **vars(opts))
    encoding = {}
    model_string = model.toprettyxml(indent='  ', **encoding)

    return model_string

def urdf_parse_from_path(model_path, args_xacro=None) -> str:
    """
    从文件路径解析 URDF 文件并返回 URDF 字符串。

    :param model_path: 模型文件的路径。
    :type model_path: str

    :param args_xacro: xacro 参数列表，例如：['arg1:=True','arg2:=10.0']
    :type args_xacro: str 列表

    :return: URDF 字符串，如果未找到文件，则返回 None。
    :rtype: str
    """

    if os.path.exists(model_path) is False:
        print("错误：模型路径不存在")
        return None

    list_args = [model_path] 
    if args_xacro is not None:
        list_args = list_args + args_xacro

    opts, input_file_name = xacro.process_args(list_args)
    model = xacro.process_file(input_file_name, **vars(opts))
    encoding = {}
    model_string = model.toprettyxml(indent='  ', **encoding)

    return model_string