# Motor sim package
### Mini challenge 3

By [afr2903](https://github.com/afr2903/)

[Challenge instructions](https://github.com/afr2903/MR3001B_Design_and_Development_of_Robots_I/blob/main/Week%203/Challenge/MCR2_DC_Motor_Sim_2.pdf)

## Summary of implementations

- URDF file of DC Motor 
- Joint State publisher node for movement of the wheel
- Sinusoidal setpoint generator
- PID controller node 
- Launch file with customizable parameters, RViz and RQT plots

## Execution

To run the main launch file:
```bash
roslaunch motor_sim motor_sim.launch _frequency:=2
```

This will launch all the nodes developed and the DC Motor in RViz will start moving.

Run `rqt_graph` to watch the node architecture.

## URDF File

The `motor_wheel` joint was tagged as `continuous` as it can rotate infinitely.

## Wheel joint state publisher node

To simulate the wheel movement in RViz, equal to the `/motor_output` topic, a conversion of units from RPMs to Radians was created and sent to the joint state.

This python node uses a `Class` structure to manage the callbacks and methods.

**Important consideration:** To achieve a smooth wheel rotation, the `Rate()` of the node needs to be high, as the `TF` package needs to update constantly. A rate of 100 hz was used. Additionally, the `joint_state_publisher_gui` should be commented in the launch file as it interferes with this node's publisher

## Setpoint generator

Simulates a sinusoidal wave, using python `math` library.

The node can receive the params `amplitude` and `frequency` in its single node execution (`rosrun`) or in the launch file.

## PID Controller

Simple PID Controller:
`output = kP * error + kI * integral + kD * derivative`

Also uses a `Class` structure for managing the callbacks and runtime methods.

Receives the params `kP`, `kI` and `kD`