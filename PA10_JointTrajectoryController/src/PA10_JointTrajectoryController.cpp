#include <cnoid/SimpleController>
#include "Interpolator.h"

/// @brief PA10に目標関節軌道を与えて動かす
class PA10_JointTrajectoryController : public cnoid::SimpleController
{
private:
    /// @brief 角度パターン一覧（計4パターン）
    std::array<Eigen::Vector<double, 9>, 4> anglePattern;

    /// @brief Bodyファイルのアクセスポインタ
    cnoid::BodyPtr ioBody;

    /// @brief 関節軌道補間器
    cnoid::Interpolator<Eigen::VectorXd> jointTrajectoryInterpolator;

    /// @brief 現在時間
    double currentTime;

    /// @brief シミュレーションステップ時間[ms]
    double timeStep;

public:
    PA10_JointTrajectoryController();
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    virtual bool control() override;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PA10_JointTrajectoryController)

PA10_JointTrajectoryController::PA10_JointTrajectoryController()
{
    // configure angle pattern array
    anglePattern[0] << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    anglePattern[1] << M_PI / 6, M_PI / 6, M_PI / 6, M_PI / 6, M_PI / 6,
        M_PI / 6, M_PI / 6, 0.015, 0.015;
    anglePattern[2] << -M_PI / 6, -M_PI / 6, -M_PI / 6, -M_PI / 6, -M_PI / 6,
        -M_PI / 6, -M_PI / 6, -0.015, -0.015;
    anglePattern[3] << M_PI / 6, -M_PI / 6, M_PI / 3, -M_PI / 3, M_PI / 6,
        -M_PI / 6, M_PI / 3, 0.015, -0.015;
}

/// @brief Executed once at SimpleController launched
/// @param io Pointer of SimpleControllerIO
/// @return Result of Initialization
bool PA10_JointTrajectoryController::initialize(cnoid::SimpleControllerIO* io)
{
    // Obtain the pointer of Body
    ioBody = io->body();

    // initialize current time
    currentTime = 0.0;

    // Obtain timestep of simulation
    timeStep = io->timeStep();

    // set sample point of joint trajectory
    jointTrajectoryInterpolator.clear();
    jointTrajectoryInterpolator.appendSample(0.0, anglePattern[0]);
    jointTrajectoryInterpolator.appendSample(2.5, anglePattern[1]);
    jointTrajectoryInterpolator.appendSample(5.0, anglePattern[2]);
    jointTrajectoryInterpolator.appendSample(7.5, anglePattern[3]);
    jointTrajectoryInterpolator.appendSample(10.0, anglePattern[0]);

    // create joint trajectory
    jointTrajectoryInterpolator.update();

    // configure each joint
    cnoid::VectorXd initialAngle =
        jointTrajectoryInterpolator.interpolate(currentTime);
    for (int i = 0; i < ioBody->numJoints(); i++)
    {
        cnoid::Link* joint = ioBody->joint(i);
        joint->setActuationMode(cnoid::Link::JointAngle);
        io->enableIO(joint);
        joint->q_target() = initialAngle[i];
    }

    return true;
}

/// @brief Executed in every period
/// @return Result of Execution
bool PA10_JointTrajectoryController::control()
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
    else
    {
        patternIndex = 3;
    }

    // Control angle of joint according to patten index
    cnoid::VectorXd jointAngle =
        jointTrajectoryInterpolator.interpolate(currentTime);
    for (int i = 0; i < ioBody->numJoints(); i++)
    {
        ioBody->joint(i)->q_target() = jointAngle[i];
    }

    // Calculate current time
    currentTime += timeStep;

    // simulate until interpolation ends
    return currentTime <= jointTrajectoryInterpolator.domainUpper();
}