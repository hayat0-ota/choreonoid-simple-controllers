#include <cnoid/SimpleController>

/// @brief ある関節指令値に動かす
class PA10_JointAngleController : public cnoid::SimpleController
{
private:
    /// @brief 軸数
    static const int jointNum = 9;

    /// @brief 角度パターン一覧（計4パターン）
    const double anglePattern[4][jointNum] = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {M_PI / 6, M_PI / 6, M_PI / 6, M_PI / 6, M_PI / 6, M_PI / 6, M_PI / 6,
         0.015, 0.015},
        {-M_PI / 6, -M_PI / 6, -M_PI / 6, -M_PI / 6, -M_PI / 6, -M_PI / 6,
         -M_PI / 6, -0.015, -0.015},
        {M_PI / 6, -M_PI / 6, M_PI / 3, -M_PI / 3, M_PI / 6, -M_PI / 6,
         M_PI / 3, 0.015, -0.015}};

    /// @brief Bodyファイルのアクセスポインタ
    cnoid::BodyPtr ioBody;

    /// @brief 現在時間
    double currentTime;

    /// @brief シミュレーションステップ時間[ms]
    double timeStep;

public:
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    virtual bool control() override;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PA10_JointAngleController)

/// @brief Executed once at SimpleController launched
/// @param io Pointer of SimpleControllerIO
/// @return Result of Initialization
bool PA10_JointAngleController::initialize(cnoid::SimpleControllerIO* io)
{
    // Obtain the pointer of Body
    ioBody = io->body();

    // configure each joint
    for (int i = 0; i < jointNum; i++)
    {
        cnoid::Link* joint = ioBody->joint(i);
        joint->setActuationMode(cnoid::Link::JointAngle);
        io->enableIO(joint);
    }

    // initialize current time
    currentTime = 0.0;

    // Obtain timestep of simulation
    timeStep = io->timeStep();

    return true;
}

/// @brief Executed in every period
/// @return Result of Execution
bool PA10_JointAngleController::control()
{
    // Select pattern index
    int patternIndex;

    if (0 <= currentTime && currentTime < 2.5)
    {
        patternIndex = 0;
    }
    else if (2.5 <= currentTime && currentTime < 5.0)
    {
        patternIndex = 1;
    }
    else if (5.0 <= currentTime && currentTime < 7.5)
    {
        patternIndex = 2;
    }
    else if (7.5 <= currentTime && currentTime < 10.0)
    {
        patternIndex = 3;
    }
    else
    {
        patternIndex = 0;
    }

    // Control angle of joint according to patten index
    for (int i = 0; i < jointNum; i++)
    {
        ioBody->joint(i)->q_target() = anglePattern[patternIndex][i];
    }

    // Calculate current time
    currentTime += timeStep;

    return true;
}