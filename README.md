# ME5413 - Homework 1: Perception
> **Author: Cao Chenyu**

(I have removed all the data files on the remote branch, but only reserved the implementations and the report.)

# Introduction
This is the repository for the homework 1 of the course ME5413: Perception. The homework is divided into three tasks:
- Task 1: Single Object Detection
- Task 2: Multiple Object Detection
- Bonus Task: Publish and Subscribe Object Detection Results

# Project Structure
```
Homework1_Perception
├─ .gitignore
├─ README.md
├─ Task 1
│  ├─ task1.ipynb
│  └─ utils
│     ├─ __init__.py
│     ├─ evaluate.py
│     ├─ load.py
│     └─ save.py
├─ Task 2
│  ├─ task2.ipynb
│  ├─ utils
│  │  ├─ __init__.py
│  │  └─ plot.py
│  └─ viz.png
├─ Bonus Task
│  ├─ bonus_task_ws
│     ├─ .catkin_workspace
│     ├─ .vscode
│     │  └─ settings.json
│     ├─ build
│     ├─ devel
│     │  ├─ lib
│     │  ├─ setup.bash
│     │  ├─ setup.sh
│     │  ├─ setup.zsh
│     ├─ run.sh
│     ├─ src
│     │  ├─ CMakeLists.txt
│     │  └─ track_node
│     │     ├─ data
│     │     │  ├─ seq_1
│     │     │  └─ seq_2
│     │     ├─ CMakeLists.txt
│     │     ├─ include
│     │     │  └─ track_node
│     │     ├─ launch
│     │     │  └─ track.launch
│     │     ├─ package.xml
│     │     ├─ scripts
│     │     │  ├─ track_pub_p.py
│     │     │  └─ track_sub_p.py
│     │     ├─ src
│     │     └─ track.bag
├─ requirements.txt
└─ ME5413_HW1_Perception.pdf
```

# Usage

## Pre-requisites

You need to create a virtual environment and install the required packages. You can do this by running the following commands:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

You have to install `ROS 1 Noetic` on `Ubuntu 20` in order to run the Bonus Task. You can install ROS 1 Noetic by following the instructions in the [official website](http://wiki.ros.org/noetic/Installation/Ubuntu).

## Task 1: Single Object Detection

Select the prepared virtual environment, then click `Run All` in the `task1.ipynb` to run the code. The code will load the dataset, perform the template matching and Kalman Filtering, and visualize all the results.

## Task 2: Multiple Object Detection

Select the prepared virtual environment, then click `Run All` in the `task2.ipynb` to run the code. The code will load the dataset, perform the object detection using different models, calculate the ADE and FDE, and visualize the results.

## Bonus Task

cd into the `bonus_task_ws` and run the following commands:

```bash
./run.sh
```
which is a shell script that will catkin_make the workspace, and roslaunch the publisher and subscriber nodes.

The exported bag file is in the `track_node` folder, and the bag file is named `track.bag`. You can play the bag file by running the following command:

```bash
rosbag play ./src/track_node/track.bag
```

# TODO list
- [x] Finish the Task 1
- [x] Finish the Task 2
- [x] Finish the Bonus Task

## Task 1: Single Object Detection
- [x] Using template matching method to track the object 
- [x] Using Kalman Filtering (based on template matching measurements) to track the object
- [x] Improve the performance of template matching by selecting the best templete based on the different dataset.
- [x] Tune the search region of template matching to improve the performance of tracking
- [x] Tune the parameters of Kalman Filtering to improve the performance of tracking, in order that using Kalman Filter would get a better performance than using only template matching.
- [ ] Adjust the width and height of the template to improve the performance of tracking 

## Task 2: Multiple Object Detection 
- [x] Using constant velocity model and calculate the ADE and FDE
- [x] Using constant acceleration model and calculate the ADE and FDE
- [x] Using CTRV model and calculate the ADE and FDE
- [x] Compare the performance of different models
- [x] Visualize the dynamic process of the object detection
- [ ] Compare the performance when using the average velocity (and acceleration) of the last 1 second frames (Because by default I use the current frame's velocity and acceleration)

## Bonus Task
- [x] Read from the bag file
- [x] Deploy the Kalman Filter Algorithm Successfully
- [x] Topic Message Publishing and Subscribing
- [x] Rosbag Recording (and Playback)
