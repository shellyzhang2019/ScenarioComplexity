#include <iostream>
#include <queue>

namespace ScenarioComplexity{
class LaneInfo
{   
    public:
    void add(const float&, const float&);
    void resize(const int&);


    std::queue<float> x;
    std::queue<float> y;
    float pointNums;
    
};

// void LaneInfo::add(const float& x_, const float& y_){
//     while(x.size() == pointNums){
//         x.pop(), y.pop();
//     }
//     x.push(x_), y.push(y_);
// }

// void LaneInfo::resize(const int& n){
//     pointNums = n;
// }

}


