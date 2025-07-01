#include "RemoteAPIClient.h"
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <thread>
#include <chrono>

// PID Controller Class
class JointSpacePIDController
{
public:
    JointSpacePIDController(RemoteAPIClient& client,
                            const std::vector<std::string>& jointNames,
                            const std::vector<float>& targetPositions,
                            const std::vector<float>& Kps,
                            const std::vector<float>& Kds,
                            const std::vector<float>& Kis,
                            float dt)
        : client_(client),
          sim_(client.getObject().sim()),
          jointNames_(jointNames),
          targetPositions_(targetPositions),
          Kps_(Kps), Kds_(Kds), Kis_(Kis),
          integralError_(jointNames.size(), 0.0f),
          dt_(dt) {}

    bool initialize()
    {
        for (const auto& name : jointNames_)
        {
            int handle = sim_.getObject(name.c_str());
            if (handle == -1)
            {
                std::cerr << "Error: Could not find joint " << name << std::endl;
                return false;
            }
            jointHandles_.push_back(handle);

            // Enable force/torque control mode
            sim_.setObjectInt32Param(handle, sim_.jointintparam_dynctrlmode, sim_.jointdynctrl_force);
        }
        return true;
    }

    void step()
    {
        std::vector<float> q(jointHandles_.size());
        std::vector<float> q_dot(jointHandles_.size());

        for (size_t i = 0; i < jointHandles_.size(); ++i)
        {
            q[i] = sim_.getJointPosition(jointHandles_[i]);
            q_dot[i] = sim_.getJointVelocity(jointHandles_[i]);
        }

        for (size_t i = 0; i < jointHandles_.size(); ++i)
        {
            float error = targetPositions_[i] - q[i];
            integralError_[i] += error * dt_;
            float derivative = -q_dot[i];

            float torque = Kps_[i] * error + Kis_[i] * integralError_[i] + Kds_[i] * derivative;

            float maxTorque = 50.0f;
            torque = std::clamp(torque, -maxTorque, maxTorque);

            printf("Joint %s | Pos: %.2f | Target: %.2f | Vel: %.2f | Torque: %.2f\n",
                   jointNames_[i].c_str(), q[i], targetPositions_[i], q_dot[i], torque);

            sim_.setJointTargetForce(jointHandles_[i], torque);
        }
    }

private:
    RemoteAPIClient& client_;
    decltype(client_.getObject().sim()) sim_;
    std::vector<std::string> jointNames_;
    std::vector<int> jointHandles_;
    std::vector<float> targetPositions_;
    std::vector<float> integralError_;
    std::vector<float> Kps_, Kds_, Kis_;
    float dt_;
};

// Main Function
int main()
{
    RemoteAPIClient client;
    auto sim = client.getObject().sim();

    std::string scenePath = "/root/planner/coppeliaSimClient/scene/Franka.ttt";
    sim.loadScene(scenePath);

    sim.setInt32Param(sim.intparam_dynamic_engine, sim.physics_mujoco);

    std::vector<std::string> jointNames = {
        "/Franka/joint", "/Franka/link2_resp/joint", "/Franka/link3_resp/joint",
        "/Franka/link4_resp/joint", "/Franka/link5_resp/joint", "/Franka/link6_resp/joint", "/Franka/link7_resp/joint"
    };

    std::vector<float> Kps = {100, 100, 100, 100, 100, 100, 50};
    std::vector<float> Kds = {10, 10, 10, 10, 5, 5, 5};
    std::vector<float> Kis = {1, 1, 1, 1, 0.5, 0.5, 0.5};

    std::vector<float> targetPositions = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.57f, 0.0f};

    float dt = 0.01f;

    JointSpacePIDController controller(client, jointNames, targetPositions, Kps, Kds, Kis, dt);

    if (!controller.initialize())
        return -1;

    sim.setStepping(true);
    sim.startSimulation();

    double t = 0.0;
    do
    {
        t = sim.getSimulationTime();
        printf("Simulation time: %.2f [s]\n", t);

        controller.step();
        sim.step();

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    } while (t < 10.0);

    sim.stopSimulation();
    return 0;
}
