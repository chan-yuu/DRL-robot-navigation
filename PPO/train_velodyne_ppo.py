#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import yaml
import torch
import numpy as np
from datetime import datetime
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecMonitor
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.logger import configure

# 导入新的环境
from velodyne_ppo_env import GazeboVelodyneEnv
from ros_utils.ros_launch import ros_kill_launch_process
import rospy
import subprocess
from utils import save_training_config

# ================= 配置参数 =================
current_dir = os.path.dirname(os.path.abspath(__file__))
config_dir = os.path.join(current_dir, 'config')
os.makedirs(config_dir, exist_ok=True)
config_path = os.path.join(config_dir, 'ppo_velodyne.yaml')

# 创建默认配置文件
default_config = {
    # 环境参数
    'environment_dim': 20,
    'max_episode_steps': 500,
    'num_envs': 4,
    'use_subproc': True,
    
    # PPO参数
    'learning_rate': 3e-4,
    'n_steps': 2048,
    'batch_size': 64,
    'n_epochs': 10,
    'gamma': 0.99,
    'gae_lambda': 0.95,
    'clip_range': 0.2,
    'clip_range_vf': None,
    'ent_coef': 0.01,
    'vf_coef': 0.5,
    'max_grad_norm': 0.5,
    'target_kl': None,
    'use_sde': False,
    'sde_sample_freq': -1,
    
    # 网络结构
    'net_arch': {'pi': [64, 64], 'vf': [64, 64]},
    'activation_fn': 'tanh',
    
    # 训练参数
    'total_timesteps': 1000000,
    'save_freq': 10000,
    'log_interval': 10,
    'verbose': 1,
    'progress_bar': True,
    'device': 'auto',
    'seed': 42,
    
    # 日志
    'tensorboard_log': True,
    'stats_window_size': 100
}

# 如果配置文件不存在，创建默认配置
if not os.path.exists(config_path):
    with open(config_path, 'w') as f:
        yaml.dump(default_config, f, default_flow_style=False)
    print(f"创建默认配置文件: {config_path}")

# 读取配置文件
with open(config_path, 'r') as f:
    config = yaml.safe_load(f)

# 合并默认配置（确保所有必要的键都存在）
for key, value in default_config.items():
    if key not in config:
        config[key] = value

# ================= 设置路径 =================
models_dir = os.path.join(current_dir, 'models')
logs_dir = os.path.join(current_dir, 'logs')
tensorboard_dir = os.path.join(current_dir, 'tensorboard_log')
os.makedirs(models_dir, exist_ok=True)
os.makedirs(logs_dir, exist_ok=True)
os.makedirs(tensorboard_dir, exist_ok=True)

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
model_name = f"PPO_Velodyne_{timestamp}"
model_save_path = os.path.join(models_dir, model_name)
tensorboard_log_dir = os.path.join(tensorboard_dir, model_name)

# 创建模型保存目录
os.makedirs(model_save_path, exist_ok=True)

# 保存配置文件到模型目录
with open(os.path.join(model_save_path, 'config.yaml'), 'w') as f:
    yaml.dump(config, f, default_flow_style=False)

def make_env(worker_index):
    """创建Velodyne环境的工厂函数"""
    def _init():
        env = GazeboVelodyneEnv(
            "multi_robot_scenario.launch", 
            config['environment_dim'],
            max_episode_steps=config['max_episode_steps'],
            pkg_name="multi_robot_scenario",
            worker_index=worker_index
        )
        
        # 设置随机种子
        if config.get('seed') is not None:
            env.seed(config['seed'] + worker_index)
        
        # 监控环境
        env = Monitor(env, os.path.join(logs_dir, f"env_{worker_index}"))
        return env
    return _init

def create_callbacks(model_save_path):
    """创建回调函数"""
    checkpoint_callback = CheckpointCallback(
        save_freq=config['save_freq'],
        save_path=os.path.join(model_save_path, "checkpoints"),
        name_prefix="ppo_velodyne",
        verbose=config['verbose']
    )
    
    return [checkpoint_callback]

def main():
    print(f"开始训练Velodyne PPO 模型: {model_name}")
    print(f"配置参数: {config}")
    print(f"使用设备: {config['device']}")
    print(f"环境数量: {config['num_envs']}")
    print("=== Velodyne导航环境特点 ===")
    print("- 使用Velodyne点云数据进行导航")
    print("- 随机目标点和障碍物位置")
    print("- 碰撞检测和目标到达奖励")
    print("- 状态空间: [激光雷达, 距离, 角度差, 线速度, 角速度]")
    
    # 保存训练配置信息
    try:
        save_training_config(config, model_name, logs_dir)
    except Exception as e:
        print(f"保存训练配置失败: {e}")
    
    # 关闭所有现有的ROS进程
    print("关闭现有的ROS进程...")
    try:
        ros_kill_launch_process()
        time.sleep(2)
    except Exception as e:
        print(f"关闭ROS进程失败: {e}")
    
    # 设置随机种子
    if config.get('seed') is not None:
        try:
            set_random_seed(config['seed'])
        except Exception as e:
            print(f"设置随机种子失败: {e}")
            return
    
    # 创建多个训练环境
    try:
        num_envs = config['num_envs']
        
        if config['use_subproc'] and num_envs > 1:
            # 使用子进程向量化环境
            env = SubprocVecEnv([
                make_env(worker_index=i) 
                for i in range(num_envs)
            ])
            print(f"创建了 {num_envs} 个子进程Velodyne环境")
        else:
            # 使用单进程向量化环境
            env = DummyVecEnv([
                make_env(worker_index=i) 
                for i in range(num_envs)
            ])
            print(f"创建了 {num_envs} 个单进程Velodyne环境")
            
        # 添加VecMonitor
        env = VecMonitor(env)
            
    except Exception as e:
        print(f"创建环境失败: {e}")
        return
    
    # 配置日志
    try:
        if config['tensorboard_log']:
            logger = configure(tensorboard_log_dir, ["stdout", "csv", "tensorboard"])
        else:
            logger = configure(logs_dir, ["stdout", "csv"])
    except Exception as e:
        print(f"配置日志失败: {e}")
        env.close()
        return
    
    # 创建PPO模型
    try:
        # 调整n_steps以适应多环境
        adjusted_n_steps = config['n_steps'] // num_envs
        if adjusted_n_steps < 1:
            adjusted_n_steps = 1
            
        model_kwargs = {
            'policy': 'MlpPolicy',  # 使用默认的MLP策略
            'env': env,
            'learning_rate': config['learning_rate'],
            'n_steps': adjusted_n_steps,
            'batch_size': config['batch_size'],
            'n_epochs': config['n_epochs'],
            'gamma': config['gamma'],
            'gae_lambda': config['gae_lambda'],
            'clip_range': config['clip_range'],
            'ent_coef': config['ent_coef'],
            'vf_coef': config['vf_coef'],
            'max_grad_norm': config['max_grad_norm'],
            'use_sde': config['use_sde'],
            'sde_sample_freq': config['sde_sample_freq'],
            'stats_window_size': config['stats_window_size'],
            'verbose': config['verbose'],
            'device': config['device'],
            'seed': config.get('seed'),
            'tensorboard_log': tensorboard_log_dir
        }
        
        # 添加可选参数
        if config['clip_range_vf'] is not None:
            model_kwargs['clip_range_vf'] = config['clip_range_vf']
        if config['target_kl'] is not None:
            model_kwargs['target_kl'] = config['target_kl']
        
        model = PPO(**model_kwargs)
        
        # 设置日志
        model.set_logger(logger)
        print(f"Velodyne PPO模型创建成功，调整后的n_steps: {adjusted_n_steps}")
        
    except Exception as e:
        print(f"创建PPO模型失败: {e}")
        env.close()
        return
    
    # 创建回调函数
    try:
        callbacks = create_callbacks(model_save_path)
    except Exception as e:
        print(f"创建回调函数失败: {e}")
        env.close()
        return
    
    # 开始训练
    try:
        print("开始训练Velodyne导航环境...")
        start_time = time.time()
        
        model.learn(
            total_timesteps=config['total_timesteps'],
            callback=callbacks,
            log_interval=config['log_interval'],
            progress_bar=config['progress_bar']
        )
        
        training_time = time.time() - start_time
        print(f"Velodyne训练完成！总用时: {training_time:.2f} 秒")
        
        # 保存最终模型
        try:
            final_model_path = os.path.join(model_save_path, "final_model")
            model.save(final_model_path)
            
            print(f"Velodyne模型已保存到: {final_model_path}")
        except Exception as e:
            print(f"保存最终模型失败: {e}")
        
        # 打印训练统计
        print("\n=== Velodyne训练统计 ===")
        print(f"总训练步数: {config['total_timesteps']}")
        print(f"训练时间: {training_time:.2f} 秒")
        print(f"平均每步时间: {training_time/config['total_timesteps']*1000:.2f} ms")
        
    except KeyboardInterrupt:
        print("训练被用户中断")
        try:
            interrupted_model_path = os.path.join(model_save_path, "interrupted_model")
            model.save(interrupted_model_path)
            print(f"模型已保存到: {interrupted_model_path}")
        except Exception as e:
            print(f"保存中断模型失败: {e}")
    
    except Exception as e:
        print(f"训练过程中出现错误: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 关闭环境
        try:
            env.close()
            print("环境已关闭")
        except Exception as e:
            print(f"关闭环境失败: {e}")

if __name__ == "__main__":
    main()