# Shallow insertion
- `shallow_insertion.m` is a matlab script to simulate inserting a card into a shallow hole.
- `moveit_shallow_insertion.py` is a python script to operate a UR10 for the insertion task. 

## Software prerequisites
- shallow_insertion.m
  * Matlab
- moveit_shallow_insertion.py
  * Ubuntu 16.04
  * ROS Kinetic
  * Moveit

## How to run it
- shallow_insertion.m
 * Open shallow_insertion.m file with matlab, click 'Run' button or enter 'Ctrl+Enter' to run the code, then you will see an insertion animation.
- moveit_shallow_insertion.py
  * rosrun moveit_tutorials moveit_shallow_insertion.py

## Result
- shallow_insertion.m
![simulation result](https://github.com/oliviaHKUST/shallow-insertion/blob/master/shallow_insertion_result.png)
