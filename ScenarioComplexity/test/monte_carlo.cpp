#include "Vehicle.h"
#include "Viz/Viewer.h"

using namespace ScenarioComplexity;

int main(){
    
    Vehicle vehicle(0, 0, 0.0, 0.0, 20, 5.0, 1, 0, 3.5, 1.8, "car");
    Viewer viewer;
    

    for(int step=0; step < 20; step++){
        vehicle.generateReachableConvex(0.1, 2);
    }

    for(int step=0; step < 20; step++){
        vehicle.generateCenterConvex(0.1, 0.0);
    }

    for(int step=0; step < 20; step++){
        vehicle.MonteCarlo(0.1, 0.01);
    }
    
    viewer.ShowConvex(vehicle.getFuturePoints(), vehicle.getReachableConvex(false), vehicle.getCenterConvex());

    return 1;
}