# turtlepath
Reinforcement learning project for CS 4033: Machine Learning.
Group members: Kevin Robb and Logan Elliott.

This project was built using ROS Melodic and Ubuntu 18.04. Our code utilizes the Stage simulator with turtlebot3. We implemented two main reinforcement learning algorithms, SARSA and Q-Learning, as well as some variations including Expected SARSA and Double-Q-Learning. We also implemented three heuristic agents for comparison: A*, potential fields, and random. The goal of our agents is to find the optimal path from a start position to a goal position on a fixed map. Our Q-Learning agents are able to learn the optimal path on a new map in around 25 episodes, and our SARSA agents can do the same in around 40 episodes. 

Our full writeup is included as a PDF.
