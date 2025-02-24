# MPC
Drag Racing Project

Project OverviewThis project focused on designing a Nonlinear Model Predictive Control (NMPC) system to optimize the distance traveled by a car in the positive x-direction within a fixed time frame. The control strategy aimed to maximize distance while maintaining vehicle stability, adhering to the centerline, and avoiding a predefined obstacle.

Key Objectives

Maximizing Distance: Ensure the vehicle covers the maximum possible distance within the allotted time.

Obstacle Avoidance: Navigate around a static obstacle located at x = 500, y = 0.

Maintaining Stability: Utilize tire dynamics to prevent loss of control and ensure smooth maneuvering.

Car DynamicsThe vehicle's motion was governed by dynamic equations that accounted for front and rear tire forces, vehicle position, and orientation in world coordinates. These equations were discretized using first-order Euler integration to enhance computational efficiency.

Modified Tire ModelTo mitigate force saturation at high slip angles, a modified tire model was introduced. This adjustment improved the optimization process, allowing the vehicle to execute precise and stable maneuvers under varying conditions.

Nonlinear Model Predictive ControlThe NMPC framework computed an optimal trajectory by balancing multiple objectives, including maximizing traction force, minimizing lateral deviation, and avoiding obstacles. Auxiliary variables were introduced to enforce tire force and friction constraints, ensuring that control inputs remained within physical limits.

Outcomes

Successfully implemented an NMPC controller that enabled the vehicle to avoid obstacles while maintaining stability.

Demonstrated the effectiveness of the modified tire model in enhancing optimization results and vehicle performance.

Future Work

Integrate additional tracking mechanisms, such as GPS, to improve positional accuracy.

Explore advanced tire models to further enhance stability and control in extreme driving conditions.

ConclusionThis project showcased the capability of NMPC in managing complex vehicle dynamics and constraints. By incorporating advanced control strategies, it contributes to the development of high-performance and autonomous driving systems.
