#include "RemoteAPIClient.h"

int main()
{
    RemoteAPIClient client;
    auto sim = client.getObject().sim();

    // Load the robot from file
    std::string scenePath = "/root/planner/coppeliaSimClient/scene/Franka.ttt";
    sim.loadScene(scenePath);

    sim.setStepping(true);

    sim.startSimulation();
    double t = 0.0;
    do
    {
        t = sim.getSimulationTime();
        printf("Simulation time: %.2f [s]\n", t);
        sim.step();
    } while (t < 3.0);
    sim.stopSimulation();
    return(0);
}