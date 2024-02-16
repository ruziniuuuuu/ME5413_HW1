# ME5413 - Homework 1: Perception
> **Author: Cao Chenyu**

# Introduction
This is the repository for the homework 1 of the course ME5413: Perception. The homework is divided into three tasks:
- Task 1: Single Object Detection
- Task 2: Multiple Object Detection
- Bonus Task

# Code Structure
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
│     └─ save_detections.py
├─ Task 2
│  ├─ task2.ipynb
│  ├─ utils
│  │  └─ __init__.py
│  └─ viz.png
├─ Bonus Task
├─ output.png
└─ requirements.txt

```

# TODO list
- [x] Finish the Task 1
- [x] Finish the Task 2
- [ ] Finish the Bonus Task

## Task 1: Single Object Detection
- [x] Using template matching method to track the object 
- [x] Using Kalman Filtering (based on template matching measurements) to track the object
- [x] Improve the performance of template matching by selecting the best templete based on the different dataset.
- [ ] Tune the parameters of Kalman Filtering to improve the performance of tracking, in order that using Kalman Filter would get a better performance than using only template matching.

## Task 2: Multiple Object Detection 
- [x] Using constant velocity model and calculate the ADE and FDE
- [x] Using constant acceleration model and calculate the ADE and FDE
- [x] Using CTRV model and calculate the ADE and FDE
- [ ] Compare the performance of different models
- [ ] Visualize the dynamic process of the object detection

## Bonus Task