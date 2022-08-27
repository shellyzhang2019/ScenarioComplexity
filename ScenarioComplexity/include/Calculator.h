#include "Scene.h"

namespace ScenarioComplexity{

class Calculator
{
private:
    int _frameAhead;
    Scene _scene;
    
public:
    Calculator();
    Calculator(const int & frame_ahead);
    ~Calculator();

    float CalcScenarioComplexity();
    float CalcCurrentDrivableArea();
};

Calculator::Calculator()
{
    _frameAhead = 20;
}

Calculator::Calculator(const int& frame_ahead)
{
    if(frame_ahead < 5){
        std::cout << "Warning too short for look ahead" << std::endl;
    }
    _frameAhead = frame_ahead;

}

Calculator::~Calculator(){};
}