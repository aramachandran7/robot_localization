# robot_localization
This is the base repo for the Olin Computational Robotics Robot Localization project
[implmentation plan](https://docs.google.com/document/d/1mB6ZcDd3plx7cEjVnaAYTViUkcwxzIZIshAOJEAcZzs/edit?usp=sharing)

## Goals
The goal of this project was to implement a particle filter algorithm in the context of robot localization. Our primary focus was learning: we wanted to write as much of the code ourselves as possible and understand the components of the codebase that we didn’t author. Our secondary goal was to have a useful particle filter that was powerful and accurate enough to be used in practice, and to incorporate some interesting experimental features along they way.

__*What is a particle filter and why do we use it in the context of robot localization?*__

A particle filter is going to be used as a method to update and obtain an accurate representation of where our robot is located in a __known__/mapped world.

This is accomplished by starting with a best guess of where the robot is at. We then make particles around that best guess that represent potential positions of the robot. Since the world has been mapped, the robot can take stock of its surroundings via lidar and compare those to a ground truth. The robot computes weights for each particle indicating how good of a match the lidar scan would be if the robot was at that position. These weights are then resampled so that the particles are centered around the more likely positions. 

As this process goes on and the robot is moving, it is exposed to more surroundings that help those particles converge to an accurate representation of the position.

## An overview of our implementation 
The implementation of a particle filter can be broken down into several steps that get repeated throughout time. Below is a high level explanation of our method. These steps assume that you understand the purpose of a particle filter, please read the above section if this is confusing.

#### Initial setup 
Initialize constants and parameters of the system
Create initial particle set around a known “best guess” pose

#### As the robot moves
- Map the robot’s movement onto each particle as if the robot was at each particle’s position
- Add noise to compensate for the robot’s odometry inaccuracy
- Scan the surroundings with the Lidar sensor
- Project the lidar scan as if it was from each different particle pose
- Assign a weight to each particle based on how similar the mapped scan is to the ground truth “map”
- Sample 85% from the particle distribution with weights as probabilities, these particles are retained
- Sample from the same distribution again, but modify this smaller subset to introduce variability (this variability is key in the event that our current particles don’t reflect the robot’s position)
- If particles have condensed too much then add variability (this helps the particle filter work as error accumulates)
- REPEAT

As this process repeats the point cloud should converge onto the actual position of the robot. Changing parameters such as the amount of resampling done or the amount of noise injected changes how the particle filter behaves.

__Here's a few clips of it in action!__
The large red arrow is the ground truth position of the robot. the small red arrows represent our particles, in theory if our filter is working they should stay relatively condensed arounf the larger red arrow. The white lines that move are the lidar scan of the robot superimposed onto the map.

![inaction](/docs/pf_inaction.gif)

![working well](/docs/in_use.gif)

Another map, this didn't work very well

![not_so_good](/docs/not_so%20good.gif)

This is the same map with a lower number of particles...

![working well](/docs/better)


[include a sexy block diagram here] ADI

#### Core design decisions explained (ADI COME THROUGH)

## Challenges
We faced a lot of problems injecting noise and variability into our filter. These original problems stemmed from some logical errors within the code. However, after straightening those out we often ended up with an overly or not aggressive particle filter. To combat this we worked to fine tune parameters but even with these changes we were unable to get the desired accuracy. 

The other signficant challenge for us was keeping track of all of the coordinate systems. These are extremely difficult to conceptualize. Luckily the tf module takes care of a lot of the work behind the scenes. This was likely the largest conceptual hurdle for us to overcome.

## Improvements
Obviously this particle filter leaves some to be desired. It works decently when given a very good estimate and a good map. To improve its accuracy we have considered several things:
- Vary the number of particles resampled based on their spread
- Improve motor model/noise
- Use more particles
- Faster timestep (more computationally intensive)
- Detect if the particle cloud has diverged and work to resolve the divergence
- Use a more versatile way of computing robot location than a weighted average of particle position

The goal of the above changes would be to produce a more accurate and usable particle filter. These changes are intended to be a list of ideas that warrant - exploration rather than a prescribed set of instructions to a better filter.

## Lessons Learned
- This was the first time we were forced to use bag files. This was extremely powerful and made testing 1000 percent faster. This is something we will carry on to future projects where applicable.
- Dividing work so that two people could be developing independently greatly increased our productivity. For example we would implement a calculation without adding noise, and then someone could work on the noise implementation and someone could work on the next calculation concurrently while still being able to test the code and avoid most merge conflicts.
- Systems quickly become too big for print statements! While we used print statements to track which case’s our code is transitioning to, visualizations or doing small tests to verify logic is a must.
