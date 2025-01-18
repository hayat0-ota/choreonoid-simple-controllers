#include <cnoid/SimpleController>
#include <random>
#include "Interpolator.h"
#include "Waypoint.h"

#pragma region Declaration

/// @brief Give joint trajectory to each joint of PA10
class PA10_JointTrajectoryController : public cnoid::SimpleController
{
private:
public:
    PA10_JointTrajectoryController();
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    virtual bool control() override;
};
CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PA10_JointTrajectoryController)

#pragma endregion

#pragma region Implementation

/// @brief Constructor (Not Implemented)
PA10_JointTrajectoryController::PA10_JointTrajectoryController() {}

/// @brief Executed once at SimpleController launched
/// @param io Pointer of SimpleControllerIO
/// @return Result of Initialization
bool PA10_JointTrajectoryController::initialize(cnoid::SimpleControllerIO* io)
{
    return true;
}

/// @brief Executed in every period
/// @return Result of Execution
bool PA10_JointTrajectoryController::control()
{
    return true;
}

#pragma endregion