#include "Geo.h"
#include "Others.h"
#include "Vehicle.h"

using namespace ScenarioComplexity;

int main(){

    Vehicle vehicle(0, 0, 0.0, 0.0, 20, 0.0, 0.0, 0, 3.5, 1.8, "car"); 

    vehicle.solve();

    return 1;
}