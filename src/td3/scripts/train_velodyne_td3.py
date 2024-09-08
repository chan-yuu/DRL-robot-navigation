#!/usr/bin/env python
import os
import time

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from numpy import inf
from torch.utils.tensorboard import SummaryWriter

from replay_buffer import ReplayBuffer  # 导入经验回放缓冲区模块
from velodyne_env import GazeboEnv  # 导入用于训练的仿真环境模块


# 评估函数，用于在给定的周期(epoch)和评估回合数(eval_episodes)下评估网络性能
def evaluate(network, epoch, eval_episodes=10):
    avg_reward = 0.0  # 平均奖励
    col = 0  # 碰撞次数计数
    for _ in range(eval_episodes):
        count = 0
        state = env.reset()  # 重置环境
        done = False
        while not done and count < 501:  # 限制最大步骤数为501
            action = network.get_action(np.array(state))  # 从网络中获取动作
            a_in = [(action[0] + 1) / 2, action[1]]  # 对动作进行调整
            state, reward, done, _ = env.step(a_in)  # 执行动作并获取下一状态、奖励等
            avg_reward += reward  # 累加奖励
            count += 1
            if reward < -90:  # 如果奖励小于-90，视为碰撞
                col += 1
    avg_reward /= eval_episodes  # 计算平均奖励
    avg_col = col / eval_episodes  # 计算平均碰撞次数
    print("..............................................")
    print(
        "Average Reward over %i Evaluation Episodes, Epoch %i: %f, %f"
        % (eval_episodes, epoch, avg_reward, avg_col)
    )
    print("..............................................")
    return avg_reward


# 定义Actor（执行者）网络，用于生成动作
class Actor(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Actor, self).__init__()

        self.layer_1 = nn.Linear(state_dim, 800)  # 第一层线性层
        self.layer_2 = nn.Linear(800, 600)  # 第二层线性层
        self.layer_3 = nn.Linear(600, action_dim)  # 输出层，用于生成动作
        self.tanh = nn.Tanh()  # 使用Tanh激活函数将动作值限制在[-1, 1]之间

    def forward(self, s):
        s = F.relu(self.layer_1(s))  # 通过ReLU激活函数处理输入
        s = F.relu(self.layer_2(s))  # 第二层ReLU激活
        a = self.tanh(self.layer_3(s))  # 通过Tanh激活生成输出动作
        return a


# 定义Critic（评论者）网络，用于评估状态-动作对的Q值
class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()

        # 第一条Q网络
        self.layer_1 = nn.Linear(state_dim, 800)
        self.layer_2_s = nn.Linear(800, 600)
        self.layer_2_a = nn.Linear(action_dim, 600)
        self.layer_3 = nn.Linear(600, 1)

        # 第二条Q网络
        self.layer_4 = nn.Linear(state_dim, 800)
        self.layer_5_s = nn.Linear(800, 600)
        self.layer_5_a = nn.Linear(action_dim, 600)
        self.layer_6 = nn.Linear(600, 1)

    def forward(self, s, a):
        # 第一条Q网络的前向传播
        s1 = F.relu(self.layer_1(s))
        self.layer_2_s(s1)
        self.layer_2_a(a)
        s11 = torch.mm(s1, self.layer_2_s.weight.data.t())
        s12 = torch.mm(a, self.layer_2_a.weight.data.t())
        s1 = F.relu(s11 + s12 + self.layer_2_a.bias.data)
        q1 = self.layer_3(s1)

        # 第二条Q网络的前向传播
        s2 = F.relu(self.layer_4(s))
        self.layer_5_s(s2)
        self.layer_5_a(a)
        s21 = torch.mm(s2, self.layer_5_s.weight.data.t())
        s22 = torch.mm(a, self.layer_5_a.weight.data.t())
        s2 = F.relu(s21 + s22 + self.layer_5_a.bias.data)
        q2 = self.layer_6(s2)
        return q1, q2


# TD3算法的网络类
class TD3(object):
    def __init__(self, state_dim, action_dim, max_action):
        # 初始化Actor网络
        self.actor = Actor(state_dim, action_dim).to(device)
        self.actor_target = Actor(state_dim, action_dim).to(device)
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters())

        # 初始化Critic网络
        self.critic = Critic(state_dim, action_dim).to(device)
        self.critic_target = Critic(state_dim, action_dim).to(device)
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters())

        self.max_action = max_action  # 动作的最大值
        self.writer = SummaryWriter()  # 用于TensorBoard的写入器
        self.iter_count = 0  # 训练迭代次数计数

    # 根据当前状态获取动作
    def get_action(self, state):
        state = torch.Tensor(state.reshape(1, -1)).to(device)
        return self.actor(state).cpu().data.numpy().flatten()

    # training cycle with additional detailed print statements
    def train(
        self,
        replay_buffer,
        iterations,
        batch_size=100,
        discount=1,
        tau=0.005,
        policy_noise=0.2,
        noise_clip=0.5,
        policy_freq=2,
    ):
        av_Q = 0
        max_Q = -inf
        av_loss = 0
        for it in range(iterations):
            # 从经验回放缓冲区中采样一个批次
            (
                batch_states,
                batch_actions,
                batch_rewards,
                batch_dones,
                batch_next_states,
            ) = replay_buffer.sample_batch(batch_size)
            
            # 将采样的状态、动作、奖励、下一状态和done打印出来
            # print(f"Batch {it}, Sampled States: {batch_states}")
            # print(f"Batch {it}, Sampled Actions: {batch_actions}")
            # print(f"Batch {it}, Sampled Rewards: {batch_rewards}")
            # print(f"Batch {it}, Sampled Next States: {batch_next_states}")
            # print(f"Batch {it}, Sampled Dones: {batch_dones}")
    
            state = torch.Tensor(batch_states).to(device)
            next_state = torch.Tensor(batch_next_states).to(device)
            action = torch.Tensor(batch_actions).to(device)
            reward = torch.Tensor(batch_rewards).to(device)
            done = torch.Tensor(batch_dones).to(device)
    
            # 使用Actor目标网络获取下一个状态的动作
            next_action = self.actor_target(next_state)
    
            # 给动作添加噪声
            noise = torch.Tensor(batch_actions).data.normal_(0, policy_noise).to(device)
            noise = noise.clamp(-noise_clip, noise_clip)
            next_action = (next_action + noise).clamp(-self.max_action, self.max_action)
    
            # 打印应用噪声后的下一步动作
            # print(f"Batch {it}, Next Action (with noise): {next_action.cpu().detach().numpy()}")
    
            # 使用Critic目标网络计算下一个状态-动作对的Q值
            target_Q1, target_Q2 = self.critic_target(next_state, next_action)
    
            # 选择较小的Q值
            target_Q = torch.min(target_Q1, target_Q2)
            av_Q += torch.mean(target_Q)
            max_Q = max(max_Q, torch.max(target_Q))
    
            # 打印目标Q值
            # print(f"Batch {it}, Target Q1: {target_Q1.cpu().detach().numpy()}")
            # print(f"Batch {it}, Target Q2: {target_Q2.cpu().detach().numpy()}")
            # print(f"Batch {it}, Min Target Q: {target_Q.cpu().detach().numpy()}")
    
            # 使用贝尔曼方程计算目标Q值
            target_Q = reward + ((1 - done) * discount * target_Q).detach()
    
            # 打印贝尔曼更新后的目标Q值
            # print(f"Batch {it}, Bellman Updated Target Q: {target_Q.cpu().detach().numpy()}")
    
            # 使用当前Critic网络计算当前Q值
            current_Q1, current_Q2 = self.critic(state, action)
    
            # 打印当前的Q值
            # print(f"Batch {it}, Current Q1: {current_Q1.cpu().detach().numpy()}")
            # print(f"Batch {it}, Current Q2: {current_Q2.cpu().detach().numpy()}")
    
            # 计算当前Q值和目标Q值之间的损失
            loss = F.mse_loss(current_Q1, target_Q) + F.mse_loss(current_Q2, target_Q)
    
            # 打印损失值
            # print(f"Batch {it}, Loss: {loss.item()}")
    
            # 进行梯度下降优化Critic网络
            self.critic_optimizer.zero_grad()
            loss.backward()
            self.critic_optimizer.step()
    
            # 打印Critic网络参数的梯度
            # for name, param in self.critic.named_parameters():
                # if param.grad is not None:
                    # print(f"Critic Param {name} Gradient: {param.grad.norm().item()}")
    
            # 每隔policy_freq次更新Actor网络
            if it % policy_freq == 0:
                actor_grad, _ = self.critic(state, self.actor(state))
                actor_grad = -actor_grad.mean()
                self.actor_optimizer.zero_grad()
                actor_grad.backward()
                self.actor_optimizer.step()
    
                # 打印Actor网络的梯度
                # for name, param in self.actor.named_parameters():
                    # if param.grad is not None:
                        # print(f"Actor Param {name} Gradient: {param.grad.norm().item()}")
    
                # 使用软更新更新Actor目标网络参数
                for param, target_param in zip(
                    self.actor.parameters(), self.actor_target.parameters()
                ):
                    target_param.data.copy_(
                        tau * param.data + (1 - tau) * target_param.data
                    )
    
                # 使用软更新更新Critic目标网络参数
                for param, target_param in zip(
                    self.critic.parameters(), self.critic_target.parameters()
                ):
                    target_param.data.copy_(
                        tau * param.data + (1 - tau) * target_param.data
                    )
    
            av_loss += loss
    
        self.iter_count += 1
    
        # 将损失和Q值写入TensorBoard，并打印信息
        self.writer.add_scalar("loss", av_loss / iterations, self.iter_count)
        # print(f"Iteration {self.iter_count}, Average Loss: {av_loss / iterations}")
    
        self.writer.add_scalar("Av. Q", av_Q / iterations, self.iter_count)
        # print(f"Iteration {self.iter_count}, Average Q: {av_Q / iterations}")
    
        self.writer.add_scalar("Max. Q", max_Q, self.iter_count)
        # print(f"Iteration {self.iter_count}, Max Q: {max_Q}")


    # 保存模型
    def save(self, filename, directory):
        torch.save(self.actor.state_dict(), "%s/%s_actor.pth" % (directory, filename))
        torch.save(self.critic.state_dict(), "%s/%s_critic.pth" % (directory, filename))

    # 加载模型
    def load(self, filename, directory):
        self.actor.load_state_dict(
            torch.load("%s/%s_actor.pth" % (directory, filename))
        )
        self.critic.load_state_dict(
            torch.load("%s/%s_critic.pth" % (directory, filename))
        )


# 设置实现的参数
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # 选择设备：cuda或cpu
seed = 0  # 随机种子号
eval_freq = 5e3  # 每隔多少步进行一次评估
max_ep = 500  # 每个回合的最大步骤数
eval_ep = 10  # 评估时的回合数
max_timesteps = 5e6  # 最大训练步数
expl_noise = 1  # 初始探索噪声值
expl_decay_steps = 500000  # 探索噪声的衰减步数
expl_min = 0.1  # 探索噪声的最小值
batch_size = 40  # mini-batch大小
discount = 0.99999  # 折扣因子
tau = 0.005  # 软更新系数
policy_noise = 0.2  # 为探索添加的噪声
noise_clip = 0.5  # 噪声的最大值
policy_freq = 2  # Actor网络更新频率
buffer_size = 1e6  # 缓冲区大小
file_name = "TD3_velodyne"  # 存储策略的文件名
save_model = True  # 是否保存模型
load_model = False  # 是否加载模型
random_near_obstacle = True  # 是否在靠近障碍物时执行随机动作

# 创建网络存储文件夹
if not os.path.exists("./results"):
    os.makedirs("./results")
if save_model and not os.path.exists("./pytorch_models"):
    os.makedirs("./pytorch_models")

# 创建训练环境
environment_dim = 20  # 环境状态维度
robot_dim = 4  # 机器人状态维度
env = GazeboEnv("multi_robot_scenario.launch", environment_dim)
time.sleep(5)
torch.manual_seed(seed)
np.random.seed(seed)
state_dim = environment_dim + robot_dim
action_dim = 2  # 动作维度
max_action = 1  # 动作的最大值

# 创建TD3网络
network = TD3(state_dim, action_dim, max_action)
# 创建经验回放缓冲区
replay_buffer = ReplayBuffer(buffer_size, seed)
if load_model:
    try:
        network.load(file_name, "./pytorch_models")
    except:
        print(
            "无法加载存储的模型参数，将使用随机参数进行训练"
        )

# 创建评估数据存储
evaluations = []

timestep = 0
timesteps_since_eval = 0
episode_num = 0
done = True
epoch = 1

count_rand_actions = 0
random_action = []


# Begin the training loop with more detailed print statements
while timestep < max_timesteps:
    print("max_timesteps: ", max_timesteps)
    print("timestep: ", timestep)
    print("timesteps_since_eval:", timesteps_since_eval)
    # 回合结束时打印回合信息
    if done:
        if timestep != 0:
            # 在回合结束时训练网络
            network.train(
                replay_buffer,
                episode_timesteps,
                batch_size,
                discount,
                tau,
                policy_noise,
                noise_clip,
                policy_freq,
            )

            # 打印当前回合的累计奖励和步数
            print(f"Episode {episode_num} finished with reward: {episode_reward}, in {episode_timesteps} steps.")
            print(f"Total timesteps: {timestep}, Episodes completed: {episode_num}")

        # 每隔一定的时间间隔进行评估并保存模型
        if timesteps_since_eval >= eval_freq:
            print(f"Validating at timestep {timestep}")
            timesteps_since_eval %= eval_freq
            evaluations.append(
                evaluate(network=network, epoch=epoch, eval_episodes=eval_ep)
            )
            network.save(file_name, directory="./pytorch_models")
            np.save("./results/%s" % (file_name), evaluations)
            epoch += 1

        # 重置环境，开始新的回合
        state = env.reset()
        done = False

        # 重置回合奖励、步数等信息
        episode_reward = 0
        episode_timesteps = 0
        episode_num += 1

    # 添加探索噪声
    if expl_noise > expl_min:
        expl_noise = expl_noise - ((1 - expl_min) / expl_decay_steps)
    
    # 每隔 1000 个 timesteps 打印当前动作和奖励信息
    if timestep % 1000 == 0:
        print(f"Timestep {timestep}: Exploring with noise: {expl_noise}")
    
    action = network.get_action(np.array(state))  # 获取动作
    action = (action + np.random.normal(0, expl_noise, size=action_dim)).clip(
        -max_action, max_action
    )

    # 如果机器人接近障碍物，则随机执行动作，以增加在障碍物附近的探索
    if random_near_obstacle:
        if (
            np.random.uniform(0, 1) > 0.85
            and min(state[4:-8]) < 0.6
            and count_rand_actions < 1
        ):
            count_rand_actions = np.random.randint(8, 15)
            random_action = np.random.uniform(-1, 1, 2)

        if count_rand_actions > 0:
            count_rand_actions -= 1
            action = random_action
            action[0] = -1

    # 将线速度范围调整到[0,1]，角速度范围调整到[-1,1]
    a_in = [(action[0] + 1) / 2, action[1]]
    next_state, reward, done, target = env.step(a_in)  # 执行动作
    done_bool = 0 if episode_timesteps + 1 == max_ep else int(done)
    done = 1 if episode_timesteps + 1 == max_ep else int(done)
    episode_reward += reward  # 累加回合奖励

    # 打印当前步骤的状态、动作、奖励、是否结束
    # print(f"Timestep {timestep}: Action: {action}, Reward: {reward}, Done: {done_bool}")
    # 每隔 100 个 timesteps 打印状态、动作和奖励信息
    if timestep % 100 == 0:
        print(f"Timestep {timestep}: Action: {action}, Reward: {reward}, Done: {done_bool}")

    # 保存元组到经验回放缓冲区
    replay_buffer.add(state, action, reward, done_bool, next_state)

    # 更新计数器
    state = next_state
    episode_timesteps += 1
    timestep += 1
    timesteps_since_eval += 1

# 训练完成后，评估并保存网络
evaluations.append(evaluate(network=network, epoch=epoch, eval_episodes=eval_ep))
if save_model:
    network.save("%s" % file_name, directory="./models")
np.save("./results/%s" % file_name, evaluations)

