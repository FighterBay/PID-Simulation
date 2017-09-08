# PID-Simulation
Simulating a PID controller on a quadcopter with random system noise.

Assumes that the thrust(motor) is directly proportional to the pwm values.

Calculating angular displacement:
angular_acceleration = torque/moment_inertia

Double integrate the above term to get angular displacement and voila?


![alt text](https://raw.githubusercontent.com/FighterBay/PID-Simulation/master/pid_graph.png)
