#!/usr/bin/env python

import rospy
from controller_manager_msgs.srv import *

def load_controller_srv(controller_name, ns=None, max_retries=5) -> bool:
    print("正在加载控制器 " + controller_name)
    if ns is not None:
        srv_name = ns + '/controller_manager/load_controller'
    else:
        srv_name = '/controller_manager/load_controller'

    rospy.wait_for_service(srv_name)
    print("等待服务")
    client_srv = rospy.ServiceProxy(srv_name, LoadController)
    try: 
        for ii in range(max_retries):
            srv_request = LoadControllerRequest(name=controller_name)
            resp1 = client_srv(srv_request)
            print("正在处理服务请求")
            if resp1.ok:
                print("请求成功")
                return True
            
        if resp1.ok is False:
            print("控制器 " + controller_name + " 无法加载。")
        
        return resp1.ok
    
    except rospy.ServiceException as exc:
        print("服务未处理请求: " + str(exc))
        return False

def load_controller_list_srv(controller_list,ns=None, max_retries=5) -> None:
    for controller in controller_list:
        load_controller_srv(controller, ns=ns, max_retries=max_retries)

def unload_controller_srv(controller_name,ns=None, max_retries=5) -> bool:
    if ns is not None:
        srv_name = ns + '/controller_manager/unload_controller'
    else:
        srv_name = '/controller_manager/unload_controller'

    rospy.wait_for_service(srv_name)
    client_srv = rospy.ServiceProxy(srv_name, UnloadController)
    
    try: 
        for ii in range(max_retries):
            srv_request = UnloadControllerRequest(name=controller_name)
            resp1 = client_srv(srv_request)
            if resp1.ok:
                return True

        if resp1.ok is False:
            print("控制器 " + controller_name + " 无法卸载。")    
        
        return resp1.ok

    except rospy.ServiceException as exc:
        print("服务未处理请求: " + str(exc))
        return False

def unload_controller_list_srv(controller_list,ns=None, max_retries=5) -> None:
    for controller in controller_list:
        unload_controller_srv(controller, ns=ns, max_retries=max_retries)

def switch_controllers_srv( start_controllers, stop_controllers, ns=None, 
                            strictness=1, start_asap=False, timeout=3.0, max_retries=5) -> bool:
    if ns is not None:
        srv_name = ns + '/controller_manager/switch_controller'
    else:
        srv_name = '/controller_manager/switch_controller'
    print("等待服务")
    rospy.wait_for_service(srv_name)
    client_srv = rospy.ServiceProxy(srv_name, SwitchController)
    
    try: 
        for ii in range(max_retries):
            print(f"正在处理服务请求: {start_controllers}")
            srv_request = SwitchControllerRequest(  start_controllers=start_controllers,stop_controllers=stop_controllers,
                                                    strictness=strictness,start_asap=start_asap,timeout=timeout)
            resp1 = client_srv(srv_request)
            if resp1.ok:
                print("请求成功")
                return True
        
        if resp1.ok is False:
            print("请求失败")
            print("控制器无法切换")

        return resp1.ok
    except rospy.ServiceException as exc:
        print("服务未处理请求: " + str(exc))
        return False

def start_controllers_srv(start_controllers, ns=None, strictness=1, start_asap=False, timeout=3.0) -> bool:

    return switch_controllers_srv(start_controllers, [], ns=ns, strictness=strictness, start_asap=start_asap, timeout=timeout)

def stop_controllers_srv(stop_controllers, ns=None, strictness=1, start_asap=False, timeout=3.0) -> bool:

    return switch_controllers_srv([], stop_controllers, ns=ns, strictness=strictness, start_asap=start_asap, timeout=timeout)

def reset_controllers_srv(reset_controllers, max_retries=10, ns=None, strictness=1, start_asap=False, timeout=3.0) -> bool:

    done_switch_off = False
    for ii in range(max_retries):
        done_switch_off = stop_controllers_srv(reset_controllers, ns=ns, strictness=strictness, start_asap=start_asap, timeout=timeout)
        if done_switch_off:
            break
    
    if not done_switch_off:
        return False

    done_switch_on = False
    for ii in range(max_retries):
        done_switch_on = start_controllers_srv(reset_controllers, ns=ns, strictness=strictness, start_asap=start_asap, timeout=timeout)
        if done_switch_on:
            break

    if not done_switch_on:
        return False
    
    return True

def spawn_controllers_srv(spawn_controllers, ns=None, strictness=1, start_asap=False, timeout=3.0) -> bool:

    load_controller_list_srv(spawn_controllers, ns=ns)

    return start_controllers_srv(spawn_controllers, ns=ns, strictness=strictness, start_asap=start_asap, timeout=timeout)

def kill_controllers_srv(kill_controllers, ns=None, strictness=1, start_asap=False, timeout=3.0) -> bool:

    res = stop_controllers_srv(kill_controllers, ns=ns, strictness=strictness, start_asap=start_asap, timeout=timeout)

    if res:
        unload_controller_list_srv(kill_controllers, ns=ns)
        return True
    else:
        return False