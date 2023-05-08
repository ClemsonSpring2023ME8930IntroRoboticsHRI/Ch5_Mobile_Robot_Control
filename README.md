# Ch5_Navigation of Mobile Robot
# Problem Statemet
Consider a scenario where robot has to reach goal by avoiding onstacle as shown in figure below.
![cpllisimu](https://user-images.githubusercontent.com/132790958/236853948-3affc714-2ae7-4256-9274-df3c169f124c.png)

Robot has to navigate toward goal by  exhibiting go to goal, obsatcle avoidance and wall following behaviors as required. 
# Pre-Requisites
Software: Coppeliasim (for simulation), Spyder (for python coding)

Basic python syntax

Understanding of the python remote API commands for coppeliasim (https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm)
# Step 1: Setting up the Scene
1.	Open CoppeliaSim and create a new scene.
2.	Add the Pioneer 3DX robot model to the scene. You can do this by dragging and dropping the model from the model library into the scene.
3.	Add a goal object to the scene. This can be any object that the Pioneer 3DX robot will navigate towards. You can use a dummy or sphere as the goal object.
4.	Add obstacles to the scene. These can be any objects that the Pioneer 3DX robot will need to avoid while navigating toward the goal. You can use boxes or cylinders as obstacles.
5.	Position the goal object and obstacles in the scene as desired.
# Step 2: Programming 
1. Add a new child script to any object in scene. You can do this by right-clicking on the robot model and selecting "Add\Script".
2. Start remote Api connection to Spyder by inputting simRemoteApi.start(19999) in child script.
3. Write the code to control the Pioneer 3DX robot to navigate towards the goal while avoiding obstacles in python using Spyder. You can use any programming language   that is supported by CoppeliaSim.
4. By using navigation architecture we developed in Chapter_5 we can control the Pioneer 3DX robot to navigate towards the goal while avoiding obstacles to showcase go to goal, avoid obstacle and wall following behavior.(check out the comments in the  python snippet attached for implementation of navigation architecture in python)
