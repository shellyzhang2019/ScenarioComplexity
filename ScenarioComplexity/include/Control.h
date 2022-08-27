#include "Others.h"

namespace ScenarioComplexity{

std::shared_ptr<Status> applyAkermannControl(const double&,
                                             const Status&, 
                                             const ControlSignal&,
                                             const Shape&);


ADStatus applyADAkermannControl(const AD<double>&, const ADStatus&, std::vector<AD<double>>&, const ADShape&);

}
