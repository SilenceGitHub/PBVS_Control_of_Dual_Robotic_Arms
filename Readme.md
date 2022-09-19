### Requirements

Matlab2021a+, Simulink, VREP 3.6.2

### Descriptions

This repository contains the **source code** of the method proposed in "**Position-Based Visual Servo Control of Dual Robotic Arms with Unknown Kinematic Models: A Cerebellum-Inspired Approach**". The three folders correspond to the three parts of the simulative studies respectively.

To run the examples, one need to open the **VREP_Scene.ttt** file in the related folders first and run the scene in VREP. Then, open the Simulink model **Main.slx** in the same folder and run it. The file **Init.m** is executed automatically for initialization when running the Simulink model. 

After the simulation is stopped, the related results will be displayed. One can also run the file **StopSim.m** to see the results if necessary.

The data of the results obtained by the authors has been saved in the folder **Data**. One can also load the file **xxxx.mat**  in the folder and run the file **StopSim.m** to see the results.
