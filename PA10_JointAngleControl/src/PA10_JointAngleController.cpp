#include <cnoid/SimpleController>
#include <random>

/// @brief ある関節指令値に動かす
class PA10_JointAngleController : public cnoid::SimpleController
{
private:
    /// @brief 軸数
    static const int jointNum = 9;

    /// @brief Joint limits of PA10 model
    const float JointLimit[jointNum] = {177.0, 94.0,  174.0, 137.0, 255.0,
                                        165.0, 255.0, 0.030, 0.030};

    /// @brief 角度パターン一覧（計4パターン）
    float anglePattern[4][jointNum];

    /// @brief Bodyファイルのアクセスポインタ
    cnoid::BodyPtr ioBody;

    /// @brief 現在時間
    double currentTime;

    /// @brief シミュレーションステップ時間[ms]
    double timeStep;

    float generateRandomFloat(float min, float max);
    float deg2rad(float deg);

public:
    PA10_JointAngleController();
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    virtual bool control() override;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PA10_JointAngleController)

/// @brief Constructor
PA10_JointAngleController::PA10_JointAngleController() {}

/// @brief Executed once at SimpleController launched
/// @param io Pointer of SimpleControllerIO
/// @return Result of Initialization
bool PA10_JointAngleController::initialize(cnoid::SimpleControllerIO* io)
{
    // Obtain the pointer of Body
    ioBody = io->body();

    // Configure each joint
    for (int i = 0; i < jointNum; i++)
    {
        cnoid::Link* joint = ioBody->joint(i);
        joint->setActuationMode(cnoid::Link::JointAngle);
        io->enableIO(joint);
    }

    // Initialize current time
    currentTime = 0.0;

    // Create joint angle patterns
    // Angle must be between joint limit.
    for (int patternId = 0; patternId < 4; patternId++)
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
    for (int i = 0; i < jointNum; i++)
    {
        // Unit must be radian
        ioBody->joint(i)->q_target() = deg2rad(anglePattern[currentPatternIndex][i]);
    }

    // Calculate current time
    currentTime += timeStep;

    return true;
}

/// @brief 適当な値を生成する
/// @param min 最小値
/// @param max 最大値
/// @return 適当に生成した値
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

/// @brief degreeからradianへの変換
/// @param angle degreeでの角度
/// @return radianでの角度
float PA10_JointAngleController::deg2rad(float deg)
{
    return deg * M_PI / 180.0f;
}