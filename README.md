# Embodiment-Dependent Risk Scenarios for Mobile Robots (EDRR)

A controlled robotics dataset and benchmark framework designed to evaluate whether AI models and risk monitors account for a robot's physical embodiment—such as size, mass, speed, and height—when assessing hazards.

## The Core Idea

When a mobile robot shares space with humans, whether a situation is dangerous depends heavily on the robot itself. For example, a small TurtleBot rounding a blind corner and bumping into an ankle poses a very different risk than a full-size humanoid robot knocking someone over. 

This project explores embodiment-aware risk assessment by:
* Executing repeatable, scripted scenarios across different robot platforms like the TurtleBot, Unitree Go1, and Unitree H2 using ROS/ROS 2.
* Recording controlled scenario families, such as blind corners and edge cases, where changing only the robot embodiment creates a drastic difference in risk.
* Providing independently reviewed ground-truth risk labels justified by physical embodiment properties rather than intuition.
* Benchmarking vision-language models on how well they adapt to changes in robot embodiment and environmental context.

See other branches for more files