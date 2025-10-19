# GSET-ROS-SIMULATION-ROBOT
Simulation of DDR (Differential Drive Robots) Using GazeboSim and RViz integrated using ROS (Robot Operating System)

This code (130,000+) have all been written for a research project at the Governor's School of Engineering and Technology over the course of two weeks in order to simulate a new PID for more accurate and applicable movement of Differential Drive Robots

At the Governor's School of Engineering and Technology, the purpose of Simulation Testing was to ensure the practical efficacy of the "Enhanced", Differential-Drive-Compatible PID, as opposed to a PID without the modifications suggested in our paper, without having to worry about many of the systematic errors associated with performing repeated experiments for months in a lab (this time was not available to our research team).

***IMPORTANT SUMMARY/MOTIVATION***
    In-depth explanation has been given below, but here, I wanted to provide some motivation for the method used to test the Enhanced PID vs. PID Controller

    To summarize, the idea is to test how the two controllers navigate an unknown map (in this case, it is a model house) by having the DDRs use their LiDAR Scanners to detect corners and edges; the idea is that, if one is unable to see in a dark room, they will cling to the corners/edges to find their room through a room, as the corners/edges of a room yield significant information about the room itself and how to exit the room. This has been accomplished with a 2D array of r-theta coordinates, finding relative maxima/minima/discontinuities [large Δr values]. Then, the robot performs Dijkstra's/A* Algorithm (if it is too close to a wall, A* is there to add a penalty term that prevents it from wandering to close to walls) to get it from the start goal to end goal

That being said, we still wanted to simulate experimental error in order to make sure it had some degree of truth to it. We implemented "error" in a few different forms, which are designed to test the limits of the Enhanced PID Controller's ability to function as a smaller part of a larger contorl system:
 1. Mechanical
 2. Optical
Note: it is also for this reason that we have implemented algorithms like Dijkstra's Algorithm and LiDAR scanning; in order to test the compatibility of the PID controller as part of a larger control system.

Mechanical error may be defined as the error caused by the movement of the system. We introduced a slight systematic error in the system in the case that the real-life analogue of a robot implementing our Enhanced PID Contorl System has this as well. Additionally, coefficients of rolling and static frictoin have been calculated and addition to the net force acting on the "TurtleBot3"-system.

Optical error may be defined as the error caused by the implementation of the LiDAR scanner on uneven surfaces. In order to simulate this movement, a random x-y "jitter value" (purposeful error) was included when simulating the beams of the LiDAR Scanner (top-half of TurtleBot3); these values are recalculated with a frequency of 10 Hz. "Error" has also been included optically through implementation of the inverse-square law of light propogation, meaning the intensity of light (and, thereby, the likelihood that the robot will receive any scattered-light from that surface) decreases. 

Finally, optical error was included by fitting the light intensity to a gaussian distribution: this distribution has been made in such a way that the front-face of the TurtleBot3 (direction of LiDAR Scanner) received the majority of the light (peak of curve), and the intensities decrease symetrically with different angles (the front-facing-axis is the axis that we choose to test angles upon). However, this error was included on the off-chance that one would want to simulate with a fixed LiDAR scanner (you are welcome to try it ourself, though it's annoying to set up).

In this case, we have given the LiDAR scanner a fixed angular velocity to gather measurements from light-scatterings upon walls in the GazeboSim software, so the "Gaussian Fitting" will have very little effect on it (it wasn't very accurate to begin with...).

Now that we have addressed the errors associated with this project, we must now address the steps taken to test the efficacy of the Enhanced PID Controller.

To start, in these nested directories, you may grep and find many ".world", ".launch", and ".py" The ".py" files (5 or so) that are responsible for testing the control over various different circumstances share a common ".." directory from the directories each of these files are located in (start in the src/ directory)

The goal of this simulation is to test the Enhanced PID Controller's efficacy when compared to a regular PID Controller for the DDR (regular PID may also be called P-controller, because the gain values that have been found were so low for "I" and "D" terms that it is hardly accurate to say "PID-controller")

Let's first begin with the tuning methods used. For both of these simulations, Zeigler Nicholas Tuning has been employed to ensure sufficient gain values in different experimental environments without arduous testing repeatedly. In this way, basic tests have been performed for the robot to perform various simple tele-operated commands - both pass with flying colors!

When each of these robots is instantiated into the model house world with the appropriate .launch file, then the robot spins in 360 degrees. As it moves in this circular manner, it only observes the light-scattering from the LiDAR Scanner to a nearby, unknown obstacle - this circling motion actually has now impact on the performance of the robot, but it can be a very helpful predictor for if the robot is working as intended (trust me, I've spent 200+ hours on simulations alone!!!)

Now, as it turns 360 degrees, the algorithm will create a range-profile: a 2D Array of (r, theta) polar coordinates that represent its current distance from a wall/object, and the angle (relative to horizontal starting-axis) at which that distance has been observed. Thereafter, the algorithm will search the array for local maxima, minima, and large changes in the values of "r": derivative dr/d(theta) is approximated, and we look for where it equals 0 or infinity (in practice, infinity is not possible, so we just iteratively check to see where the value of r_i - r_i-1 suddenly increases).

Afterwards, another, far smaller 2D array is constructed from points that have passed this test.



