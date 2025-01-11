#include <cnoid/SimpleController>
#include <random>

#pragma region Declaration

/// @brief Give joint angle to each joint of PA10
class PA10_JointAngleController : public cnoid::SimpleController
{
private:
    /// @brief The number of joint
    static const int jointNum = 9;

    /// @brief  The number of pattern
    static const int patternNum = 4;

    /// @brief Joint limits of PA10 model
    const float JointLimit[jointNum] = {177.0, 94.0,  174.0, 137.0, 255.0,
                                        165.0, 255.0, 0.030, 0.030};

    /// @brief Pattern of joint angles
    float anglePattern[patternNum][jointNum];

    /// @brief Pointer for Body
    cnoid::BodyPtr ioBody;

    /// @brief Curremt simulation time [ms]
    double currentTime;

    /// @brief Simulation step time [ms]
    double timeStep;

    /// Methods
    float generateRandomFloat(float min, float max);
    float deg2rad(float deg);

public:
    /// Methods
    PA10_JointAngleController();
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    virtual bool control() override;
};
CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PA10_JointAngleController)

#pragma endregion

#pragma region Implementation

/// @brief Constructor (Not Implemented)
PA10_JointAngleController::PA10_JointAngleController() {}

/// @brief Executed once at SimpleController launched
/// @param io Pointer of SimpleControllerIO
/// @return Result of Initialization
bool PA10_JointAngleController::initialize(cnoid::SimpleControllerIO* io)
{
    // Obtain the pointer of Body
    ioBody = io->body();

    // Configure each joint
    for (int jointId = 0; jointId < jointNum; jointId++)
    {
        cnoid::Link* joint = ioBody->joint(jointId);

        // Set the way to control joint
        joint->setActuationMode(cnoid::Link::JointAngle);

        // Enable Input and Output for joint
        io->enableIO(joint);
    }

    // Initialize current time
    currentTime = 0.0;

    // Create joint angle patterns
    // Angle must be between joint limit.
    for (int patternId = 0; patternId < patternNum; patternId++)
    {
        for (int jointId = 0; jointId < jointNum; jointId++)
        {
            anglePattern[patternId][jointId] =
                generateRandomFloat(-JointLimit[jointId], JointLimit[jointId]);
        }
    }

    // Obtain timestep of simulation
    timeStep = io->timeStep();

    return true;
}

/// @brief Executed in every period
/// @return Result of Execution
bool PA10_JointAngleController::control()
{
    // Select pattern index according to current simulation time
    int currentPatternIndex;
    if (0 <= currentTime && currentTime < 2.5)
        currentPatternIndex = 0;
    else if (2.5 <= currentTime && currentTime < 5.0)
        currentPatternIndex = 1;
    else if (5.0 <= currentTime && currentTime < 7.5)
        currentPatternIndex = 2;
    else if (7.5 <= currentTime && currentTime < 10.0)
        currentPatternIndex = 3;
    else
        currentPatternIndex = 0;

    // Control angle of joint according to patten index
    for (int jointId = 0; jointId < jointNum; jointId++)
    {
        // Unit must be radian
        ioBody->joint(jointId)->q_target() = deg2rad(anglePattern[currentPatternIndex][jointId]);
    }

    // Calculate current time
    currentTime += timeStep;

    return true;
}

/// @brief Generate random float value
/// @param min Minimum
/// @param max Maximum
/// @return Generated value
float PA10_JointAngleController::generateRandomFloat(float min, float max)
{
    if (min > max)
    {
        throw std::invalid_argument("min must be less than or equal to max");
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(min, max);

    return dis(gen);
}

/// @brief Convert from degree to radian
/// @param angle Angle in degree
/// @return Angle in radian
float PA10_JointAngleController::deg2rad(float deg)
{
    return deg * M_PI / 180.0f;
}

#pragma endregion