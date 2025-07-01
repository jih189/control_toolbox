# Joint Space PID Controller (Franka Emika Panda)

This section implements a **Joint Space PID Controller** in C++ for the Franka Emika Panda robot.

The controller operates in **torque control mode** and uses per-joint proportional, derivative, and integral gains to track target joint positions.

---

## 🚀 What This Code Does

- Loads a `.ttt` scene containing the Franka robot.
- Sets each joint into **torque control mode** using `sim.jointdynctrl_force`.
- Reads joint positions and velocities every simulation step.
- Applies torques based on a **per-joint PID controller**:
```
τ_i = Kp[i] * (q_des[i] - q[i]) + Ki[i] * ∫(q_des[i] - q[i]) dt - Kd[i] * q_dot[i]
```
- Steps the simulation in **manual stepping mode**, with a fixed timestep.

## Run
In one terminal, you can launch the simulator
```
coppeliaSim.sh
```

In other terminal, you run the following code
```
cd /root/planner/coppeliaSimClient/build
./jointspace_pid_controller
```