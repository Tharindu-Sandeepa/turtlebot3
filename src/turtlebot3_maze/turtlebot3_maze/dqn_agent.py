#!/usr/bin/env python3
"""
Deep Q-Network Agent for TurtleBot3 Maze Navigation
"""

import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random
from collections import deque

class DQN(nn.Module):
    def __init__(self, state_size, action_size, hidden_size=128):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, hidden_size)
        self.fc4 = nn.Linear(hidden_size, action_size)
        
        self.relu = nn.ReLU()
        
    def forward(self, x):
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        x = self.relu(self.fc3(x))
        return self.fc4(x)

class DQNAgent:
    def __init__(self, state_size, action_size, learning_rate=0.001, gamma=0.99,
                 epsilon=1.0, epsilon_min=0.01, epsilon_decay=0.995,
                 memory_size=10000, batch_size=32):
        
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=memory_size)
        self.batch_size = batch_size
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_decay
        self.learning_rate = learning_rate
        
        # Device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Networks
        # Initialize Q-network for training
        self.q_network = DQN(state_size, action_size).to(self.device)
        # Initialize target network for stable Q-value estimates
        self.target_network = DQN(state_size, action_size).to(self.device)
        # Setup Adam optimizer for Q-network training
        self.optimizer = optim.Adam(self.q_network.parameters(), lr=learning_rate)
        
        # Copy initial weights from Q-network to target network
        self.update_target_network()
        
    def update_target_network(self):
        self.target_network.load_state_dict(self.q_network.state_dict())
        
    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
        
    def act(self, state):
        # Epsilon-greedy action selection
        # With probability epsilon, choose a random action for exploration
        if np.random.random() <= self.epsilon:
            return random.randrange(self.action_size)
        
        # Otherwise, choose the action with highest Q-value (exploitation)
        # Convert state to tensor and add batch dimension
        state = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        # Get Q-values from network
        q_values = self.q_network(state)
        # Return action with maximum Q-value
        return np.argmax(q_values.cpu().data.numpy())
    
    def replay(self):
        if len(self.memory) < self.batch_size:
            return
        
        # Sample a random minibatch from replay memory
        batch = random.sample(self.memory, self.batch_size)
        # Convert batch elements to tensors and move to the appropriate device
        states = torch.FloatTensor([e[0] for e in batch]).to(self.device)
        actions = torch.LongTensor([e[1] for e in batch]).to(self.device)
        rewards = torch.FloatTensor([e[2] for e in batch]).to(self.device)
        next_states = torch.FloatTensor([e[3] for e in batch]).to(self.device)
        dones = torch.BoolTensor([e[4] for e in batch]).to(self.device)
        
        # Get current Q-values for the actions actually taken (shape: batch_size x 1)
        current_q_values = self.q_network(states).gather(1, actions.unsqueeze(1))
        # Compute next state's Q-values using the target network and detach from graph
        next_q_values = self.target_network(next_states).max(1)[0].detach()
        # Compute target Q-values: reward + gamma * next_q * (1 - done)
        # Using bitwise NOT on dones (~dones) yields True for non-terminal steps,
        # which acts as a mask to zero out next_q_values for terminal states.
        target_q_values = rewards + (self.gamma * next_q_values * ~dones)
        
        # Compute loss between current Q and target Q, then perform a gradient step
        loss = nn.MSELoss()(current_q_values.squeeze(), target_q_values)
        
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
    
    def save(self, filename):
        torch.save(self.q_network.state_dict(), filename)
        
    def load(self, filename):
        self.q_network.load_state_dict(torch.load(filename))
        self.update_target_network()
