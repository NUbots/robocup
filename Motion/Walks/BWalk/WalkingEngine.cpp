/**
* @file WalkingEngine.cpp
* Implementation of a module that creates the walking motions
* @author Colin Graf
*/

#include <cstdio>
#include <assert.h>
#include <cmath>

#include "WalkingEngine.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Tools/Math/General.h"
#include "Kinematics/NUInverseKinematics.h"
#include "Requirements/MassCalibration.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Requirements/ForwardKinematic.h"
#include "NUPlatform/NUPlatform.h"
#include "debug.h"
#include "debugverbositynumotion.h"

//#include "Tools/InverseKinematic.h"
//#include "Tools/MessageQueue/InMessage.h"
//#include "Tools/Streams/InStreams.h"
//#include "Tools/Settings.h"
//#include "Tools/Math/Matrix.h"
//#include "Platform/SoundPlayer.h"

void WalkingEngine::breakPointIndicator(){

}

inline float saveAsinh(float xf)
{
  double x = xf; // yes, we need double here
#ifdef _MSC_VER
  return float(log(x + sqrt(x * x + 1.)));
#else
  return float(asinh(x));
#endif
}

inline float saveAcosh(float xf)
{
  //assert(xf >= 1.f);
  double x = xf; // yes, we need double here
  if(x < 1.)
    return 0.000001f;
#ifdef WIN32
  x = log(x + sqrt(x * x - 1.));
#else
  x = acosh(x);
#endif
  if(x < 0.000001)
    return 0.000001f;
  return float(x);
}

template<typename T> ostream& operator<<(ostream& output, const Vector3<T>& v)
{
    output << "[";
    output << v.x << ",";
    output << v.y << ",";
    output << v.z;
    output << "]";
    return output;
}

template<typename T> ostream& operator<<(ostream& output, const Vector2<T>& v)
{
    output << "[";
    output << v.x << ",";
    output << v.y;
    output << "]";
    return output;
}

WalkingEngine::KickPlayer::KickType WalkingEngine::getKickType( Vector2<> position, Vector2<> target){
    if (false/*!m_setup_kick*/){
        //testing by commenting this out
        m_kick_type = KickPlayer::none;
        return m_kick_type;
    }

    if(m_recalculate_kick_type) {
#if DEBUG_NUMOTION_VERBOSITY > 2
            debug << __PRETTY_FUNCTION__ << ": m_recalculate_kick_type == true;" << endl;
#endif

        float ball_x = position.x;//Relative coords
        float ball_y = position.y;

        float target_x = target.x;
        float target_y = target.y;
        //Check for division by zero or zero vectors:
        if (target_x - ball_x == 0 and target_y == ball_y) {
            cout << __PRETTY_FUNCTION__ << ": ball postition and target position identical" << endl;
            m_recalculate_kick_type = false;
            m_kick_type = KickPlayer::none;
            return m_kick_type;
        }

        double theta;
        if(target_x-ball_x!=0){
            theta = atan2(target_y - ball_y, target_x - ball_x);
        } else {
            if(target_y>ball_y){
                //delta_y positive
                theta = mathGeneral::PI /2.0;
            }else{
                //delta_y negative
                theta = -mathGeneral::PI /2.0;
            }
        }

        // triggers sidekick too often with -45 deg to 45 deg front kick zone
        float angle_margin = mathGeneral::PI / 2.0f;

        if(theta > angle_margin)
        {
            m_kick_type = KickPlayer::sidewardsRight;
            cout << __PRETTY_FUNCTION__ << ": kick type set to sidewardsLeft"<< endl;
        }
        else if(theta <= angle_margin and theta >= -angle_margin and ball_y >= 0)
        {
            m_kick_type = KickPlayer::left;
            cout << __PRETTY_FUNCTION__ << ": kick type set to left"<< endl;
        }
        else if(theta < -angle_margin)
        {
            m_kick_type = KickPlayer::sidewardsLeft;
            cout << __PRETTY_FUNCTION__ << ": kick type set to sidewardsRight"<< endl;
        }
        else if(theta >= -angle_margin and theta <= angle_margin and ball_y < 0)
        {
            m_kick_type = KickPlayer::right;
            cout << __PRETTY_FUNCTION__ << ": kick type set to right"<< endl;
        }
        else
        {
            std::cout << __PRETTY_FUNCTION__ 
                      << ": No kick available for position: (" 
                      << ball_x << ", " << ball_y << ")" << std::endl;
            m_kick_type = KickPlayer::none;
        }
        m_recalculate_kick_type = false;
    } 
    
    return m_kick_type;
}

WalkingEngine::WalkingEngine(NUSensorsData* data, NUActionatorsData* actions, NUInverseKinematics* ik) : NUWalk(data,actions), emergencyShutOff(false), currentMotionType(stand),
    m_ik(ik), instable(true), beginOfStable(0), m_kick_type(KickPlayer::none),kickPlayer()
{
  observedPendulumPlayer.walkingEngine = this;

  // default values for parameters not in the cfg file
  p.standBikeRefX = 20.f;
  p.standStandbyRefX = 3.f;
  //p.standComPosition = Vector3<>(/*20.5f*/ 3.5f /*0.f*/, 50.f, /*259.f*/ /*261.5f*/ 258.0f);
  //p.standComPosition = Vector3<>(3.5f, 50.f,258.0f);
  p.standComPosition = Vector3<>(3.5f, 50.f, 160.0f);
  p.standBodyTilt = 0.03f; //01f;
  p.standArmJointAngles = Vector2<>(0.2f, 0.f);

  p.standHardnessAnklePitch = 75;
  p.standHardnessAnkleRoll = 75;

  //p.walkRefX = 15.f;
//  p.walkRefX = 10.f;
  //p.walkRefXAtFullSpeedX = 7.f;
//  p.walkRefXAtFullSpeedX = 3.f;

//  p.walkRefY = 50.f;
  //p.walkRefY = 40.f;
  //p.walkRefYAtFullSpeedX = 38.f;
//  p.walkRefYAtFullSpeedY = 50.f;
  //p.walkRefYAtFullSpeedY = 40.f;
  //p.walkStepDuration = 470.f;
  //p.walkStepDurationAtFullSpeedX = 400.f;
  //p.walkStepDurationAtFullSpeedY = 400.f;

  p.walkHeight = Vector2<>(p.standComPosition.z, 300.f);
  p.walkArmRotation = 0.4f;
  p.walkRefXSoftLimit.min = -3.f;
  p.walkRefXSoftLimit.max = 5.f;
  p.walkRefXLimit.min = -15.f;
  p.walkRefXLimit.max = 13.f;
  p.walkRefYLimit.min = -3.f;
  p.walkRefYLimit.max = 3.f;
  p.walkRefYLimitAtFullSpeedX.min = -30.f;
  p.walkRefYLimitAtFullSpeedX.max = 30.f;

  //p.walkLiftOffset = Vector3<>(0.f, -5.f, 17.f);
  //p.walkLiftOffset = Vector3<>(0.f, -5.f, 25.f);
  //p.walkLiftOffsetJerk = 0.f;
  //p.walkLiftOffsetAtFullSpeedY = Vector3<>(0.f, -10.f, 27.f);
  //p.walkLiftRotation = Vector3<>(-0.05f, -0.05f, 0.f);
  //p.walkAntiLiftOffset = Vector3<>(0.f, 0.f, 2.3f);
  //p.walkAntiLiftOffsetAtFullSpeedY = Vector3<>(0.f, 0.f, 2.3f);

  //p.walkComBodyRotation = 0.05f;
  p.walkFadeInShape = Parameters::sine;

  p.kickComPosition = Vector3<>(20.f, 0.f, 245.f);
  p.kickX0Y = 1.f;
  p.kickHeight = Vector2<>(p.standComPosition.z, 300.f);

  //p.speedMax = Pose2D(0.6f, 60.f * 2.f, 50.f);
  //p.speedMaxMin = Pose2D(0.2f, 10.f, 0.f);
  //p.speedMaxBackwards = 50 * 2.f;
  //p.speedMaxChange = Pose2D(0.3f, 8.f, 20.f);

  //p.observerMeasurementMode = Parameters::torsoMatrix;
  p.observerMeasurementMode = Parameters::robotModel;
  p.observerMeasurementDelay = 40.f;

  p.observerErrorMode = Parameters::mixed;
  p.observerProcessDeviation = Vector4f(0.1f, 0.1f, 3.f, 3.f);
  p.observerMeasurementDeviation = Vector2f(20.f, 20.f);
  p.observerMeasurementDeviationAtFullSpeedX = Vector2f(20.f, 20.f);
  p.observerMeasurementDeviationWhenInstable = Vector2f(20.f, 10.f);

  p.balance = true;
  //p.balance = false;
  p.balanceMinError = Vector3<>(0.f, 0.f, 0.f);
  p.balanceMaxError = Vector3<>(8.f, 8.f, 8.f);
  //p.balanceCom.x = PIDCorrector::Parameters(0.1f, 0.2f, 0.1f, 2.f); // known good value

  p.balanceCom.x = PIDCorrector::Parameters(0.11f, 0.0f, 0.0f, 30.f);
  p.balanceCom.y = PIDCorrector::Parameters(0.11f, 0.0f, 0.0f, 30.f);
  p.balanceCom.z = PIDCorrector::Parameters(0.11f, 0.0f, 0.0f, 30.f);
  p.balanceCom.y = PIDCorrector::Parameters(0.f, 0.0f, 0.f, 0.f);

  p.balanceBodyRotation.x = PIDCorrector::Parameters(0.f, 0.0f, 0.f, 30.f);
  p.balanceBodyRotation.y = PIDCorrector::Parameters(0.f, 0.0f, 0.f, 30.f);
  p.balanceStepSize = Vector2<>(0.08f, -0.04f);
  p.balanceStepSizeWhenInstable = Vector2<>(0.16f, -0.06f);
  p.balanceStepSizeWhenPedantic = Vector2<>(0.04f, -0.02f);
  p.balanceStepSizeInterpolation = 0.1f;

  p.stabilizerOnThreshold = 200.f;
  p.stabilizerOffThreshold = 100.f;
  p.stabilizerDelay = 200;

  p.odometryUseTorsoMatrix = true;
  p.odometryScale = Pose2D(1.f, 1.2f, 1.f);
  p.odometryUpcomingScale = Pose2D(1.f, 1.8f, 1.2f);
  p.odometryUpcomingOffset = Pose2D(0.f, 0.f, 0.f);

  bodyToCom = Vector3<>(0,0,0);
  
  m_walk_parameters.load("BWalk");
  writeParameters();
  
  init();
}

WalkingEngine::~WalkingEngine()
{
    return;
}

void WalkingEngine::init()
{
  // load parameters from config file
    p.computeContants();
#ifdef TARGET_SIM
  p.observerMeasurementDelay = 60.f;
#endif

  currentRefX = p.standStandbyRefX;
  balanceStepSize = p.balanceStepSize;
  m_prev_time = m_data->CurrentTime;
  m_cycle_time = 0;
  m_initialised = false;
  m_walk_requested = false;
  m_pedantic = false;

  float halfArmRotation = p.walkArmRotation * 0.5f;

  // Write initial positions.
  m_initial_larm.resize(m_actions->getSize(NUActionatorsData::LArm),0.0f);
  m_initial_larm[0] = p.standArmJointAngles.x;
  m_initial_larm[1] = -(-mathGeneral::PI/2.0f + p.standArmJointAngles.y);
  m_initial_larm[2] = -p.standArmJointAngles.y - halfArmRotation;
  if(m_initial_larm.size() > 3)
  {
    m_initial_larm[3] = -mathGeneral::PI/2.0f;
  }

  m_initial_rarm.resize(m_actions->getSize(NUActionatorsData::RArm),0.0f);
  m_initial_rarm[0] = -p.standArmJointAngles.x;
  m_initial_rarm[1] = -(-mathGeneral::PI/2.0f + p.standArmJointAngles.y);
  m_initial_rarm[2] = -(-p.standArmJointAngles.y - halfArmRotation);
  if(m_initial_larm.size() > 3)
  {
    m_initial_rarm[3] = -(-mathGeneral::PI/2.0f);
  }

  std::vector<float> joints(m_actions->getSize(NUActionatorsData::All),0.0f);

  Matrix leftTarget(4,4,true);

//  leftTarget[0][3] = p.standStandbyRefX+10;
//  leftTarget[1][3] = p.standComPosition.y;
//  leftTarget[2][3] = -p.standComPosition.z + 30.0f;
//  Matrix rightTarget(4,4,true);
//  rightTarget[0][3] = p.standStandbyRefX+10;
//  rightTarget[1][3] = -p.standComPosition.y;
//  rightTarget[2][3] = -p.standComPosition.z + 30.0f;
//  m_ik->calculateLegJoints(leftTarget, rightTarget, joints);

//  std::vector<float>::iterator l_leg_start = joints.begin() + m_actions->getSize(NUActionatorsData::Head) + m_actions->getSize(NUActionatorsData::LArm) + m_actions->getSize(NUActionatorsData::RArm);
//  std::vector<float>::iterator l_leg_end = l_leg_start + m_actions->getSize(NUActionatorsData::LLeg);
//  std::vector<float>::iterator r_leg_start = l_leg_end;
//  std::vector<float>::iterator r_leg_end = r_leg_start + m_actions->getSize(NUActionatorsData::RLeg);

  m_initial_lleg.resize(m_actions->getSize(NUActionatorsData::LLeg),0.f);
  m_initial_lleg[1] = -0.5;
  m_initial_lleg[3] = 1.2;
  m_initial_lleg[5] = -0.7;

  m_initial_rleg = m_initial_lleg;

  nu_nextLeftArmJoints.resize(m_actions->getSize(NUActionatorsData::LArm), 0.0f);   // Left Arm
  nu_nextRightArmJoints.resize(m_actions->getSize(NUActionatorsData::RArm), 0.0f);  // Right Arm
  nu_nextLeftLegJoints.resize(m_actions->getSize(NUActionatorsData::LLeg), 0.0f);   // Left Leg
  nu_nextRightLegJoints.resize(m_actions->getSize(NUActionatorsData::RLeg), 0.0f);  // Right Leg
}

void WalkingEngine::setWalkParameters(const WalkParameters& walkparameters)
{
#if DEBUG_NUMOTION_VERBOSITY > 0
    debug << "WalkingEngine::setWalkParameters: " << endl;
#endif
    m_walk_parameters = walkparameters;
    writeParameters();
}

void WalkingEngine::writeParameters()
{
#if DEBUG_NUMOTION_VERBOSITY > 0
        debug << "WalkingEngine::writeParameters: " << endl;
#endif
    vector<Parameter>& params = m_walk_parameters.getParameters();
    for(unsigned int i=0; i<params.size(); i++) {
        string& nm = params.at(i).name();
        float value = params.at(i).get();
#if DEBUG_NUMOTION_VERBOSITY > 0
        debug << nm << " : " << value << endl;
#endif
        if(nm.compare("standComPositionZ") == 0)
            p.standComPosition.z = value;
        else if(nm.compare("walkRefX") == 0)
            p.walkRefX = value;
        else if(nm.compare("walkRefXAtFullSpeedX") == 0)
            p.walkRefXAtFullSpeedX = value;
        else if(nm.compare("walkRefY") == 0)
            p.walkRefY = value;
        else if(nm.compare("walkRefYAtFullSpeedX") == 0)
            p.walkRefYAtFullSpeedX = value;
        else if(nm.compare("walkRefYAtFullSpeedY") == 0)
            p.walkRefYAtFullSpeedY = value;
        else if(nm.compare("walkStepDuration") == 0)
            p.walkStepDuration = value;
        else if(nm.compare("walkStepDurationAtFullSpeedX") == 0)
            p.walkStepDurationAtFullSpeedX = value;
        else if(nm.compare("walkStepDurationAtFullSpeedY") == 0)
            p.walkStepDurationAtFullSpeedY = value;
        else if(nm.compare("walkLiftOffset0") == 0)
            p.walkLiftOffset.x = value;
        else if(nm.compare("walkLiftOffset1") == 0)
            p.walkLiftOffset.y = value;
        else if(nm.compare("walkLiftOffset2") == 0)
            p.walkLiftOffset.z = value;
        else if(nm.compare("walkLiftOffsetJerk") == 0)
            p.walkLiftOffsetJerk = value;
        else if(nm.compare("walkLiftOffsetAtFullSpeedY0") == 0)
            p.walkLiftOffsetAtFullSpeedY.x = value;
        else if(nm.compare("walkLiftOffsetAtFullSpeedY1") == 0)
            p.walkLiftOffsetAtFullSpeedY.y = value;
        else if(nm.compare("walkLiftOffsetAtFullSpeedY2") == 0)
            p.walkLiftOffsetAtFullSpeedY.z = value;
        else if(nm.compare("walkLiftRotation0") == 0)
            p.walkLiftRotation.x = value;
        else if(nm.compare("walkLiftRotation1") == 0)
            p.walkLiftRotation.y = value;
        else if(nm.compare("walkLiftRotation2") == 0)
            p.walkLiftRotation.z = value;
        else if(nm.compare("walkAntiLiftOffset0") == 0)
            p.walkAntiLiftOffset.x = value;
        else if(nm.compare("walkAntiLiftOffset1") == 0)
            p.walkAntiLiftOffset.y = value;
        else if(nm.compare("walkAntiLiftOffset2") == 0)
            p.walkAntiLiftOffset.z = value;
        else if(nm.compare("walkAntiLiftOffsetAtFullSpeedY0") == 0)
            p.walkAntiLiftOffsetAtFullSpeedY.x = value;
        else if(nm.compare("walkAntiLiftOffsetAtFullSpeedY1") == 0)
            p.walkAntiLiftOffsetAtFullSpeedY.y = value;
        else if(nm.compare("walkAntiLiftOffsetAtFullSpeedY2") == 0)
            p.walkAntiLiftOffsetAtFullSpeedY.z = value;
        else if(nm.compare("walkComBodyRotation") == 0)
            p.walkComBodyRotation = value;
        else if(nm.compare("speedMaxRot") == 0)
            p.speedMax.rotation = value;
        else if(nm.compare("speedMaxX") == 0)
            p.speedMax.translation.x = value;
        else if(nm.compare("speedMaxY") == 0)
            p.speedMax.translation.y = value;
        else if(nm.compare("speedMaxMinRot") == 0)
            p.speedMaxMin.rotation = value;
        else if(nm.compare("speedMaxMinX") == 0)
            p.speedMaxMin.translation.x = value;
        else if(nm.compare("speedMaxMinY") == 0)
            p.speedMaxMin.translation.y = value;
        else if(nm.compare("speedMaxBackwards") == 0)
            p.speedMaxBackwards = value;
        else if(nm.compare("speedMaxChangeRot") == 0)
            p.speedMaxChange.rotation = value;
        else if(nm.compare("speedMaxChangeX") == 0)
            p.speedMaxChange.translation.x = value;
        else if(nm.compare("speedMaxChangeY") == 0)
            p.speedMaxChange.translation.y = value;
        else if(nm.compare("balanceComXP") == 0)
            p.balanceCom.x.p = value;
        else if(nm.compare("balanceComXD") == 0)
            p.balanceCom.x.d = value;
        else if(nm.compare("balanceComYP") == 0)
            p.balanceCom.y.p = value;
        else if(nm.compare("balanceComYD") == 0)
            p.balanceCom.y.d = value;
        else if(nm.compare("balanceComZP") == 0)
            p.balanceCom.z.p = value;
        else if(nm.compare("balanceComZD") == 0)
            p.balanceCom.z.d = value;
        else if(nm.compare("balanceBodyRotationXP") == 0)
            p.balanceBodyRotation.x.p = value;
        else if(nm.compare("balanceBodyRotationXD") == 0)
            p.balanceBodyRotation.x.d = value;
        else if(nm.compare("balanceBodyRotationYP") == 0)
            p.balanceBodyRotation.y.p = value;
        else if(nm.compare("balanceBodyRotationYD") == 0)
            p.balanceBodyRotation.y.d = value;
        else
            debug << "WalkingEngine::setWalkParameters(): No matching parameter found: " << nm << endl;
    }

    // Copy the default NUwalk parameter speed limits into the B-Human walk speed limits.
    std::vector<float> max_speeds = m_walk_parameters.getMaxSpeeds();
    p.speedMax.translation.x = max_speeds[0] * 10.0f;   // Conversion from cm to mm
    p.speedMax.translation.y = max_speeds[1] * 10.0f;   // Conversion from cm to mm
    p.speedMax.rotation = max_speeds[2];

    //must do this
    p.computeContants();
}

void WalkingEngine::doWalk()
{

#if DEBUG_NUMOTION_VERBOSITY > 2
        debug << "WalkingEngine::doWalk()"<<endl;
#endif
    static std::vector<float> joints(m_actions->getSize(NUActionatorsData::All), 0.0f);

//    m_cycle_time = 0.001 * (m_data->CurrentTime - m_prev_time);
//    m_prev_time = m_data->CurrentTime;
    m_cycle_time = 0.02; // time in seconds. Added for dynamic kick (Jake)
    bool validJoints = m_data->getPosition(NUSensorsData::All, joints);
    if(validJoints)
    {
#if DEBUG_NUMOTION_VERBOSITY > 2
            debug << "WalkingEngine::doWalk() setting joint data"<<endl;
#endif
        theRobotModel.setJointData(joints, theMassCalibration);
#if DEBUG_NUMOTION_VERBOSITY > 2
            debug << "WalkingEngine::doWalk() joint data set, updating"<<endl;
#endif
        update();
    }
    else
    {
        std::cout << "Joints Invalid" << std::endl;
    }
    return;
}


void WalkingEngine::update(/*WalkingEngineOutput& walkingEngineOutput*/)
{
#if DEBUG_NUMOTION_VERBOSITY > 2
        debug << "WalkingEngine::update() 1"<<endl;
#endif
    bool walking = true;
    bool careful = false;
    m_pedantic = careful;

    float delta_time = m_data->CurrentTime - m_prev_time;
    if(delta_time > (25 * m_cycle_time * 1000))
    {
        m_speed_yaw = 0.0f;
        m_speed_x = 0.0f;
        m_speed_y = 0.0f;
        requestedWalkTarget = Pose2D(0, 0, 0);
        requestedMotionType = stand;
        balanceStepSize = p.balanceStepSize;
    }
#if DEBUG_NUMOTION_VERBOSITY > 2
        debug << "WalkingEngine::update() 2"<<endl;
#endif
    m_prev_time = m_data->CurrentTime;
    
    if(walking/*theMotionSelection.ratios[MotionRequest::walk] > 0.f || theMotionSelection.ratios[MotionRequest::stand] > 0.f*/)
    {
        //Vector2<> targetBalanceStepSize = theMotionRequest.walkRequest.pedantic ? p.balanceStepSizeWhenPedantic : p.balanceStepSize;
        Vector2<> targetBalanceStepSize = m_pedantic ? p.balanceStepSizeWhenPedantic : p.balanceStepSize;
        Vector2<> step = targetBalanceStepSize - balanceStepSize;
        Vector2<> maxStep(std::abs(balanceStepSize.x - p.balanceStepSizeWhenPedantic.x) * p.balanceStepSizeInterpolation,
                          std::abs(balanceStepSize.y - p.balanceStepSizeWhenPedantic.y) * p.balanceStepSizeInterpolation);
        if(step.x > maxStep.x)
            step.x = maxStep.x;
        else if(step.x < -maxStep.x)
            step.x = -maxStep.x;
        if(step.y > maxStep.y)
            step.y = maxStep.y;
        else if(step.y < -maxStep.y)
            step.y = -maxStep.y;
        
        balanceStepSize += step;

#if DEBUG_NUMOTION_VERBOSITY > 2
            debug << "WalkingEngine::update() doing updateMotionRequest()"<<endl;
#endif
        updateMotionRequest();

#if DEBUG_NUMOTION_VERBOSITY > 2
            debug << "WalkingEngine::update() doing updateObservedPendulumPlayer()"<<endl;
#endif
        updateObservedPendulumPlayer();

#if DEBUG_NUMOTION_VERBOSITY > 2
            debug << "WalkingEngine::update() doing computeMeasuredStance()"<<endl;
#endif
        computeMeasuredStance();

#if DEBUG_NUMOTION_VERBOSITY > 2
            debug << "WalkingEngine::update() doing computeExpectedStance()"<<endl;
#endif
        computeExpectedStance();

#if DEBUG_NUMOTION_VERBOSITY > 2
            debug << "WalkingEngine::update() doing computeError()"<<endl;
#endif
        computeError();

#if DEBUG_NUMOTION_VERBOSITY > 2
            debug << "WalkingEngine::update() doing  updatePendulumPlayer()"<<endl;
#endif
        updatePendulumPlayer();

#if DEBUG_NUMOTION_VERBOSITY > 2
            debug << "WalkingEngine::update() doing updateKickPlayer()"<<endl;
#endif
        updateKickPlayer();

#if DEBUG_NUMOTION_VERBOSITY > 2
            debug << "WalkingEngine::update() doing generateTargetStance()"<<endl;
#endif
        generateTargetStance();

#if DEBUG_NUMOTION_VERBOSITY > 2
            debug << "WalkingEngine::update() doing generateJointRequest()"<<endl;
#endif
        generateJointRequest();

#if DEBUG_NUMOTION_VERBOSITY > 2
            debug << "WalkingEngine::update() doing computeOdometryOffset()"<<endl;
#endif
        computeOdometryOffset();

#if DEBUG_NUMOTION_VERBOSITY > 2
            debug << "WalkingEngine::update() doing generateOutput()"<<endl;
#endif
        generateOutput(/*walkingEngineOutput*/);

#if DEBUG_NUMOTION_VERBOSITY > 2
            debug << "WalkingEngine::update() done "<<endl;
#endif
    }
    else
    {
#if DEBUG_NUMOTION_VERBOSITY > 2
            debug << "WalkingEngine::update() done"<<endl;
#endif
        currentMotionType = stand;
        /*
        if(theMotionSelection.ratios[MotionRequest::specialAction] >= 1.f)
          if(theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead || theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::sitDown)
            currentRefX = p.standStandbyRefX;
            */
        generateDummyOutput(/*walkingEngineOutput*/);
    }
    m_setup_kick = false;
}

void WalkingEngine::updateMotionRequest()
{
//old code
//  if(theMotionRequest.motion == MotionRequest::walk)
//  {
//    if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
//    {
//      if(theMotionRequest.walkRequest.target != Pose2D())
//        requestedWalkTarget = theMotionRequest.walkRequest.target;
//    }
//    else
//      requestedWalkTarget = theMotionRequest.walkRequest.speed; // just for sgn(requestedWalkTarget.translation.y)
//  }

    // Get values from the NUWalk speeds.
    requestedWalkTarget = Pose2D(m_target_speed_yaw, 10*m_target_speed_x, 10*m_target_speed_y);
//    requestedWalkTarget = Pose2D(1.0, 0, 0);
    requestedMotionType = stand;
    if(getKickType(m_ball_position, m_ball_target)!= KickPlayer::none && kickPlayer.isKickStandKick(getKickType( m_ball_position, m_ball_target)))
    {
      bool mirrored = kickPlayer.isKickMirrored(getKickType( m_ball_position, m_ball_target));
      requestedMotionType = mirrored ? standLeft : standRight;
      m_walk_requested = false;
    }
    else if(!kickPlayer.isActive() && (fabs(m_target_speed_x) > 0.05 or fabs(m_target_speed_y) > 0.05 or fabs(m_target_speed_yaw) > 0.001))
    {
        m_walk_requested = true;
        if(!instable)
        {
            requestedMotionType = stepping;
        }
    }
    else
    {
        m_walk_requested = false;
    }

//    std::cout << " Requested Motion Type: ";
//    if(requestedMotionType == stepping) std::cout << "Stepping";
//    else if(requestedMotionType == stand) std::cout << "Stand";
//    else std::cout << "Unknown";
//    std::cout << std::endl << "instable = " << (instable?"True":"False") << std::endl;
    
  // get requested motion state
//  requestedMotionType = stand;
//  if((theGroundContactState.contactSafe || !theDamageConfiguration.useGroundContactDetectionForSafeStates) && !theWalkingEngineOutput.enforceStand && theMotionSelection.ratios[MotionRequest::walk] > 0.999f && !instable)
//    if(theMotionRequest.motion == MotionRequest::walk)
//    {


}

void WalkingEngine::updateObservedPendulumPlayer()
{
  // motion update
  if(observedPendulumPlayer.isActive())
    observedPendulumPlayer.seek(m_cycle_time);

  // change motion type
  switch(currentMotionType)
  {
  case stand:
  case standLeft:
  case standRight:
    if(kickPlayer.isActive())
      break;
#if DEBUG_NUMOTION_VERBOSITY > 2
    debug<<"WalkingEngine::updateObservedPendulumPlayer() - 1st swritch current motion standRight case"<<endl;
#endif
    if(requestedMotionType != currentMotionType)
    {
      SupportLeg supportLeg = SupportLeg(0);
      Vector2<> r;
      Vector2<> x0;
      Vector2<> k = p.walkK;
      StepType stepType = fromStand;
      switch(currentMotionType)
      {
      case standRight:

        //assert(false);
#if DEBUG_NUMOTION_VERBOSITY > 2
        debug<<"WalkingEngine::updateObservedPendulumPlayer() - standRight case"<<endl;
#endif
        supportLeg = right;
        r = Vector2<>(0.f, -(p.standComPosition.y - p.kickComPosition.y + p.kickX0Y));
        x0 = Vector2<>(currentRefX, p.kickX0Y);
        k = p.kickK;
        stepType = fromStandLeft;
        break;
      case standLeft:
           debug<<"WalkingEngine::updateObservedPendulumPlayer() - standLeft case"<<endl;
        //assert(false);
        supportLeg = left;
        r = Vector2<>(0.f, p.standComPosition.y - p.kickComPosition.y + p.kickX0Y);
        x0 = Vector2<>(currentRefX, -p.kickX0Y);
        k = p.kickK;
        stepType = fromStandRight;
        break;
      case stand:
#if DEBUG_NUMOTION_VERBOSITY > 2
           debug<<"WalkingEngine::updateObservedPendulumPlayer() - stand case"<<endl;
#endif
           breakPointIndicator();
        if(requestedMotionType == standRight || (requestedMotionType == stepping && requestedWalkTarget.translation.y > 0.f))
        {
          supportLeg = left;
          r = Vector2<>(currentRefX, p.walkRefY);
          x0 = Vector2<>(0.f, -p.walkRefY);
        }
        else
        {
          supportLeg = right;
          r = Vector2<>(currentRefX, -p.walkRefY);
          x0 = Vector2<>(0.f, p.walkRefY);
        }
        breakPointIndicator();
        break;
      default:
#if DEBUG_NUMOTION_VERBOSITY > 2
        debug<<"WalkingEngine::updateObservedPendulumPlayer() - default case: asserting false"<<endl;
#endif
        assert(false);
        break;
      }
// #if DEBUG_NUMOTION_VERBOSITY > 2
//        debug<<"WalkingEngine::updateObservedPendulumPlayer() - 1"<<endl;
// #endif
      lastNextSupportLeg = supportLeg;
// #if DEBUG_NUMOTION_VERBOSITY > 2
//       debug<<"WalkingEngine::updateObservedPendulumPlayer() - 2"<<endl;
// #endif
      lastSelectedSpeed = Pose2D();
// #if DEBUG_NUMOTION_VERBOSITY > 2
//       debug<<"WalkingEngine::updateObservedPendulumPlayer() - 3"<<endl;
// #endif
      nextPendulumParameters.s = StepSize();
// #if DEBUG_NUMOTION_VERBOSITY > 2
//       debug<<"WalkingEngine::updateObservedPendulumPlayer() - 4"<<endl;
// #endif
      observedPendulumPlayer.init(stepType, p.observerMeasurementDelay * -0.001f, supportLeg, r, x0, k, m_cycle_time);
// #if DEBUG_NUMOTION_VERBOSITY > 2
//       debug<<"WalkingEngine::updateObservedPendulumPlayer() - 5"<<endl;
// #endif
      currentMotionType = stepping;
// #if DEBUG_NUMOTION_VERBOSITY > 2
//       debug<<"WalkingEngine::updateObservedPendulumPlayer() - 6"<<endl;
// #endif
    }
    break;
  default:
    break;
  }
}

void WalkingEngine::computeMeasuredStance()
{
    float heightLeg5Joint = 45.19;
    std::vector<float> orientation_sensors;
    bool validKinematics = m_data->getOrientation(orientation_sensors);

//    std::vector<float> accelerations;
//    m_data->getAccelerometer(accelerations);
//    float roll = mathGeneral::PI/2.f + atan2(accelerations[2], accelerations[1]);
//    float pitch = mathGeneral::PI/2.f + atan2(accelerations[2], accelerations[0]);


    std::vector<float> accelerations;
    m_data->getAccelerometer(accelerations);
    Vector3<> accel(accelerations[0], accelerations[1], accelerations[2]);
    float roll = -asin(accel.y / accel.abs());
    float pitch = asin(accel.x / accel.abs());


//    std::cout << "pitch: " << pitch << std::endl;
//    std::cout << "roll: " << roll << std::endl;
//    std::cout << "accel: [" << accelerations[0] << ", " << accelerations[1] << ", " << accelerations[2] << "]" << std::endl;
//    std::cout << "orientation: [" << orientation_sensors[0] << ", " << orientation_sensors[1] << "]" << std::endl;
    const Vector3<> acc_axis(roll, pitch, 0);
    const Vector3<> axis(orientation_sensors[0], orientation_sensors[1], 0);
    RotationMatrix torso(acc_axis);

  switch(p.observerMeasurementMode)
  {
  case Parameters::robotModel:
  {

    Pose3D comToLeft = Pose3D(-theRobotModel.centerOfMass).conc(theRobotModel.limbs[MassCalibration::footLeft]).translate(0.f, 0.f, -heightLeg5Joint);
    Pose3D comToRight = Pose3D(-theRobotModel.centerOfMass).conc(theRobotModel.limbs[MassCalibration::footRight]).translate(0.f, 0.f, -heightLeg5Joint);
    Pose3D leftToCom = comToLeft.invert();
    Pose3D rightToCom = comToRight.invert();

    RotationMatrix rotationLeft(Vector3<>(
                                  atan2(leftToCom.rotation.c1.z, leftToCom.rotation.c2.z),
                                  atan2(-leftToCom.rotation.c0.z, leftToCom.rotation.c2.z),
                                  0.f));
    RotationMatrix rotationRight(Vector3<>(
                                   atan2(rightToCom.rotation.c1.z, rightToCom.rotation.c2.z),
                                   atan2(-rightToCom.rotation.c0.z, rightToCom.rotation.c2.z),
                                   0.f));

    RotationMatrix& bodyRotation = observedPendulumPlayer.supportLeg == left ? rotationLeft : rotationRight;
    // TODO: optimize

    measuredLeftToCom = -Pose3D(bodyRotation).conc(comToLeft).translation;
    measuredRightToCom = -Pose3D(bodyRotation).conc(comToRight).translation;
  }
  break;

  default:
    measuredLeftToCom = -Pose3D(torso).translate(-theRobotModel.centerOfMass).conc(theRobotModel.limbs[MassCalibration::footLeft]).translate(0.f, 0.f, -heightLeg5Joint).translation;
    measuredRightToCom = -Pose3D(torso).translate(-theRobotModel.centerOfMass).conc(theRobotModel.limbs[MassCalibration::footRight]).translate(0.f, 0.f, -heightLeg5Joint).translation;
    break;
  }
}

void WalkingEngine::computeExpectedStance()
{
  LegStance* expectedStance;
  if(legStances.getNumberOfEntries() == 0)
  {
    legStances.add();
    expectedStance = &legStances.getEntry(0);
    getStandStance(*expectedStance);
    stepOffset = StepSize();
  }
  else
  {
    int index = std::min(int(p.observerMeasurementDelay / 10.f - 0.5f), legStances.getNumberOfEntries() - 1);
    expectedStance = &legStances.getEntry(index);
    if(observedPendulumPlayer.isActive() && !observedPendulumPlayer.isLaunching())
      observedPendulumPlayer.getStance(*expectedStance, 0, 0, &stepOffset);
    else
    {
      stepOffset = StepSize();
    }
  }

  expectedLeftToCom = expectedStance->leftOriginToCom - expectedStance->leftOriginToFoot.translation;
  expectedRightToCom = expectedStance->rightOriginToCom - expectedStance->rightOriginToFoot.translation;
}

void WalkingEngine::computeError()
{
    bool fallen = m_data->isFallen();
    bool falling = m_data->isFalling();

//  if((theGroundContactState.contactSafe || !theDamageConfiguration.useGroundContactDetectionForSafeStates) && theFallDownState.state != FallDownState::onGround && theFallDownState.state != FallDownState::falling)
  if(!fallen && !falling)
  {
    leftError = measuredLeftToCom - expectedLeftToCom;
    rightError = measuredRightToCom - expectedRightToCom;

//    std::cout << "leftError: " << leftError << "; measuredLeftToCom: " << measuredLeftToCom << "; expectedLeftToCom: " << expectedLeftToCom << std::endl;
//    std::cout << "rightError: " << rightError << "; measuredRightToCom: " << measuredRightToCom << "; expectedRightToCom: " << expectedRightToCom << std::endl;

    // I'm not sure what this part does
//    if(theMotionSelection.ratios[MotionRequest::walk] < 0.99f)
//    {
//      instability.add(((leftError + rightError) * 0.5f).squareAbs());
//      instable = true;
//    }
//    else if(kickPlayer.isActive())
//    {
//      instability.add((observedPendulumPlayer.supportLeg == left ? leftError : rightError).squareAbs());
//    }
//    else
//    {
      instability.add(((leftError + rightError) * 0.5f).squareAbs());
      if(instability.getAverage() > p.stabilizerOnThreshold)
      {
        instable = true;
        beginOfStable = 0;
      }
      else if(instability.getAverage() < p.stabilizerOffThreshold)
      {
        if(!beginOfStable)
          beginOfStable = m_data->CurrentTime;
        else if((m_data->CurrentTime - beginOfStable) > p.stabilizerDelay)
          instable = false;
      }
  }
    for(int i = 0; i < 2; ++i)
    {
      Vector3<>& error = i == 0 ? leftError : rightError;
      for(int i = 0; i < 3; ++i)
      {
        if(error[i] > -p.balanceMinError[i] && error[i] < p.balanceMinError[i])
          error[i] = 0.f;
        else if(error[i] > p.balanceMinError[i])
          error[i] -= p.balanceMinError[i];
        else
          error[i] += p.balanceMinError[i];

        if(error[i] > p.balanceMaxError[i])
          error[i] = p.balanceMaxError[i];
        else if(error[i] < -p.balanceMaxError[i])
          error[i] = -p.balanceMaxError[i];
      }
    }
}

void WalkingEngine::updatePendulumPlayer()
{
  if(currentMotionType == stepping)
  {
    if(p.balance)
      observedPendulumPlayer.applyCorrection(leftError, rightError, m_cycle_time);
    cout<<"void WalkingEngine::updatePendulumPlayer() - beg p.te = "<< p.te<<endl;
    cout<<"void WalkingEngine::updatePendulumPlayer() - beg pendulumPlayer.te = "<< pendulumPlayer.tb<<" "<<pendulumPlayer.te<<" "<<pendulumPlayer.t<<" "<<endl;
    pendulumPlayer = observedPendulumPlayer;
    pendulumPlayer.seek(p.observerMeasurementDelay * 0.001f);

    if(!pendulumPlayer.isActive())
    {
      currentRefX = pendulumPlayer.next.r.x;
      switch(pendulumPlayer.type)
      {
      case toStand:
        currentMotionType = stand;
        break;
      case toStandLeft:
        currentMotionType = standLeft;
        break;
      case toStandRight:
        currentMotionType = standRight;
        break;
      default:
        break;
      }
    }
  }
  //cout<<"void WalkingEngine::updatePendulumPlayer() -end p.te = "<< p.te<<endl;
  //cout<<"void WalkingEngine::updatePendulumPlayer() -end pendulumPlayer.te = "<< pendulumPlayer.tb<<" "<<pendulumPlayer.te<<" "<<pendulumPlayer.t<<" "<<endl;
}

void WalkingEngine::updateKickPlayer()
{
// #if DEBUG_NUMOTION_VERBOSITY > 2
  debug << __PRETTY_FUNCTION__ << ": start" << endl;
// #endif

  // If the robot is stepping (instead of stand, standLeft, or standRight)
  if(currentMotionType == stepping)
  {
    // If not currently kicking, but a kick is planned
    if(!kickPlayer.isActive() && pendulumPlayer.kickType != KickPlayer::none)
    {
      // initialize the kick
// #if DEBUG_NUMOTION_VERBOSITY > 2
      debug << __PRETTY_FUNCTION__ << ": (!kickPlayer.isActive() && pendulumPlayer.kickType != KickPlayer::none) so init kickPlayer"<<endl;
// #endif
      kickPlayer.init(pendulumPlayer.kickType, m_ball_position, m_ball_target);
    }

    // If the robot is kicking/ready to kick
    if(kickPlayer.isActive())
    {
      if(kickPlayer.getType() != pendulumPlayer.kickType)
      {
        // Stop the kickplayer's kick (i.e. go with the pendulum player's choice)
// #if DEBUG_NUMOTION_VERBOSITY > 2
        debug << __PRETTY_FUNCTION__ << ": kickPlayer.isActive() and (kickPlayer.getType() != pendulumPlayer.kickType) so stop kickPlayer"<<endl;
// #endif
        kickPlayer.stop();
      }
      else // If the kickPlayer and pendulumPlayer agree
      {
// #if DEBUG_NUMOTION_VERBOSITY > 2
        debug << __PRETTY_FUNCTION__ << ": !kickPlayer.isActive() so getLength"<<endl;
// #endif

        // Get the length of the kick 'motion'?
        float length = kickPlayer.getLength();

        if(length < 0.f) 
          cout<< "WARNING: kickPlayer.length is negative!"<< endl;

        // std::cout <<__PRETTY_FUNCTION__ << ": " << "p.te = " << p.te << endl;
        std::cout << __PRETTY_FUNCTION__ << ": "
                  << std::endl << "    "
                  << "length: " << length
                  << std::endl << "    "
                  << "pendulumPlayer: { "
                  <<   "tb: " << pendulumPlayer.tb
                  << ", te: " << pendulumPlayer.te
                  << ", t:"   << pendulumPlayer.t
                  << " }"
                  << std::endl;

        // Use the length and elapsed time to calculate at which position 
        // in the kick motion the robot should be.
        // (Note: A position here is a pose?)
        float pos = length * (pendulumPlayer.t - pendulumPlayer.tb) / (pendulumPlayer.te - pendulumPlayer.tb);


// #if DEBUG_NUMOTION_VERBOSITY > 2
        debug     <<__PRETTY_FUNCTION__ << ": "
                  << std::endl << "    "
                  << "length: " << length
                  << std::endl << "    "
                  << "pendulumPlayer: { "
                  <<   "tb: " << pendulumPlayer.tb
                  << ", te: " << pendulumPlayer.te
                  << ", t:"   << pendulumPlayer.t
                  << " }"
                  << std::endl;
        debug << __PRETTY_FUNCTION__ << ": !kickPlayer.isActive() calculate pos = "<< pos <<endl;

        debug << __PRETTY_FUNCTION__ << ": !kickPlayer.isActive() seek"<<endl;
// #endif

        // Move the robot's pose forward in time to the appropriate position
        // in the kick script.
        kickPlayer.seek(std::max(pos - kickPlayer.getCurrentPosition(), 0.f));

// #if DEBUG_NUMOTION_VERBOSITY > 2
        debug << __PRETTY_FUNCTION__ << ": !kickPlayer.isActive() end seek"<<endl;
// #endif
      }
    }
  }
  else // If the robot is not currently stepping
  {
    // If the robot is kicking/ready to kick
    if(kickPlayer.isActive())
    {
// #if DEBUG_NUMOTION_VERBOSITY > 2
      debug << __PRETTY_FUNCTION__ << ": kickplayer.seek"<<endl;
// #endif
      kickPlayer.seek(m_cycle_time);
    } 
    else if(KickPlayer::KickType(getKickType( m_ball_position, m_ball_target)) != KickPlayer::none 
            && currentMotionType == requestedMotionType 
            && (requestedMotionType == standLeft || requestedMotionType == standRight))
    {
// #if DEBUG_NUMOTION_VERBOSITY > 2
      debug << __PRETTY_FUNCTION__ << ": kickplayer.init"<<endl;
// #endif
      kickPlayer.init( getKickType( m_ball_position, m_ball_target),  m_ball_position, m_ball_target);
    }
  }
// #if DEBUG_NUMOTION_VERBOSITY > 2
  debug << __PRETTY_FUNCTION__ << ": finish"<<endl;
// #endif
}

void WalkingEngine::generateTargetStance()
{
  std::vector<float> headPos;
  if(m_data->getPosition(NUSensorsData::Head,headPos))
  {
      targetStance.headJointAngles[0] = headPos[0];//theHeadJointRequest.pan;
      targetStance.headJointAngles[1] = headPos[1];//theHeadJointRequest.tilt;
  }

  float leftArmAngle = 0.f, rightArmAngle = 0.f;
  if(currentMotionType == stepping)
    pendulumPlayer.getStance(targetStance, &leftArmAngle, &rightArmAngle, 0);
  else
    getStandStance(targetStance);

  // set arm angles
  float halfArmRotation = p.walkArmRotation * 0.5f;
  targetStance.leftArmJointAngles[0] = -(-mathGeneral::PI/2.0f + p.standArmJointAngles.y + leftArmAngle);
  targetStance.leftArmJointAngles[1] = p.standArmJointAngles.x;
  targetStance.leftArmJointAngles[2] = -mathGeneral::PI/2.0f;
  targetStance.leftArmJointAngles[3] = -p.standArmJointAngles.y - leftArmAngle - halfArmRotation;
  targetStance.rightArmJointAngles[0] = -(-mathGeneral::PI/2.0f + p.standArmJointAngles.y + rightArmAngle);
  targetStance.rightArmJointAngles[1] = -(p.standArmJointAngles.x);
  targetStance.rightArmJointAngles[2] = -(-mathGeneral::PI/2.0f);
  targetStance.rightArmJointAngles[3] = -(-p.standArmJointAngles.y - rightArmAngle - halfArmRotation);

  // playing a kick motion!?
  //KICKDEBUG - check if statement
  if(kickPlayer.isActive())
  {
      cout<< "WalkingEngine::generateTargetStance() - kick is active - applying kickstance "<<endl;
    kickPlayer.setParameters( m_ball_position, m_ball_target);
    kickPlayer.apply(targetStance);
  }

  legStances.add(targetStance);
}

void WalkingEngine::getStandStance(LegStance& stance) const
{
  stance.leftOriginToFoot = Pose3D(Vector3<>(0.f, p.standComPosition.y, 0.f));
  stance.rightOriginToFoot = Pose3D(Vector3<>(0.f, -p.standComPosition.y, 0.f));
  if(currentMotionType == stand)
  {
    stance.leftOriginToCom = stance.rightOriginToCom = Vector3<>(p.standComPosition.x + currentRefX, 0.f, p.standComPosition.z);
    //stance.leftOriginToCom = stance.rightOriginToCom = Vector3<>(p.standComPosition.x + p.standStandbyRefX, 0.f, p.standComPosition.z);
  }
  else
  {
    const float sign = currentMotionType == standLeft ? 1.f : -1.f;
    stance.leftOriginToCom =  stance.rightOriginToCom = Vector3<>(p.kickComPosition.x + currentRefX, (p.standComPosition.y - p.kickComPosition.y) * sign, p.kickComPosition.z);
  }
}

void WalkingEngine::generateJointRequest()
{
  Vector3<> correctedLeftOriginToCom = targetStance.leftOriginToCom;
  Vector3<> correctedRightOriginToCom = targetStance.rightOriginToCom;

  if(p.balance)
  {
    correctedLeftOriginToCom.x += leftControllerX.getCorrection(leftError.x, m_cycle_time, p.observerMeasurementDelay, p.balanceCom.x);
    correctedLeftOriginToCom.y += leftControllerY.getCorrection(leftError.y, m_cycle_time, p.observerMeasurementDelay, p.balanceCom.y);
    correctedLeftOriginToCom.z += leftControllerZ.getCorrection(leftError.z, m_cycle_time, p.observerMeasurementDelay, p.balanceCom.z);
    correctedRightOriginToCom.x += rightControllerX.getCorrection(rightError.x, m_cycle_time, p.observerMeasurementDelay, p.balanceCom.x);
    correctedRightOriginToCom.y += rightControllerY.getCorrection(rightError.y, m_cycle_time, p.observerMeasurementDelay, p.balanceCom.y);
    correctedRightOriginToCom.z += rightControllerZ.getCorrection(rightError.z, m_cycle_time, p.observerMeasurementDelay, p.balanceCom.z);
  }

  if(currentMotionType == stepping)
  {
    if(pendulumPlayer.supportLeg == left)
    {
      if(pendulumPlayer.l.z != 0.f)
        correctedRightOriginToCom.z -= p.walkLiftOffsetJerk;
    }
    else
    {
      if(pendulumPlayer.l.z != 0.f)
        correctedLeftOriginToCom.z -= p.walkLiftOffsetJerk;
    }
  }


  const float heightLeg5Joint = 45.19;

  std::vector<float> joint_positions;
  m_data->getPosition(NUSensorsData::All, joint_positions);
  unsigned int total_arm_joints = m_actions->getSize(NUActionatorsData::LArm);

  std::vector<float>::iterator l_arm_it = joint_positions.begin() + m_actions->getSize(NUActionatorsData::Head);
  std::vector<float>::iterator l_arm_end = l_arm_it + m_actions->getSize(NUActionatorsData::LArm);
  std::vector<float>::iterator r_arm_it = l_arm_end;
  std::vector<float>::iterator r_arm_end = r_arm_it + m_actions->getSize(NUActionatorsData::RArm);

  *(l_arm_it++) = targetStance.leftArmJointAngles[1];
  *(l_arm_it++) = targetStance.leftArmJointAngles[0];
  *(l_arm_it++) = targetStance.leftArmJointAngles[3];

  if(total_arm_joints > 3)
  {
    *(l_arm_it++) = targetStance.leftArmJointAngles[2];
  }

  assert(l_arm_it == l_arm_end);

  *(r_arm_it++) = targetStance.rightArmJointAngles[1];
  *(r_arm_it++) = targetStance.rightArmJointAngles[0];
  *(r_arm_it++) = -targetStance.rightArmJointAngles[3];
  if(total_arm_joints > 3)
  {
      *(r_arm_it-1) = -*(r_arm_it-1);
      *(r_arm_it++) = targetStance.rightArmJointAngles[2];
  }

  assert(r_arm_it == r_arm_end);

  float bodyRotationX = 0.f, bodyRotationY = 0.f;
  if(p.balance)
  {
    bodyRotationY = atan(bodyControllerX.getCorrection((leftError.x + rightError.x) * 0.5f, m_cycle_time, p.observerMeasurementDelay, p.balanceBodyRotation.x) / p.walkHeight.x);
    bodyRotationX = atan(bodyControllerY.getCorrection((leftError.y + rightError.y) * 0.5f, m_cycle_time, p.observerMeasurementDelay, p.balanceBodyRotation.y) / p.walkHeight.y);
  }

  float additionalBodyRotation = (((targetStance.rightOriginToCom.y - targetStance.rightOriginToFoot.translation.y) - p.standComPosition.y) + ((targetStance.leftOriginToCom.y - targetStance.leftOriginToFoot.translation.y) + p.standComPosition.y)) * 0.5f;
  additionalBodyRotation *= 1.f / (22.5f - 50.f);
  additionalBodyRotation *= p.walkComBodyRotation;
  RotationMatrix bodyRotation(Vector3<>(additionalBodyRotation + bodyRotationX, bodyRotationY, 0.f));
  bodyRotation *= p.standBodyRotation;

  const Pose3D comToLeftOrigin = Pose3D(bodyRotation, correctedLeftOriginToCom).invert();
  const Pose3D comToRightOrigin = Pose3D(bodyRotation, correctedRightOriginToCom).invert();

  // TODO: optimize this by calculating the inverted left/rightOriginToCom pose directly
  const Pose3D comToLeftAnkle = Pose3D(comToLeftOrigin).conc(targetStance.leftOriginToFoot).translate(0.f, 0.f, heightLeg5Joint);
  const Pose3D comToRightAnkle = Pose3D(comToRightOrigin).conc(targetStance.rightOriginToFoot).translate(0.f, 0.f, heightLeg5Joint);
  const Vector3<> averageComToAnkle = (comToLeftAnkle.translation + comToRightAnkle.translation) * 0.5f;

  Vector3<> bodyToCom = this->bodyToCom;
  Vector3<> bodyToComOffset = lastAverageComToAnkle != Vector3<>() ? (averageComToAnkle - lastAverageComToAnkle) * 0.4f : Vector3<>();
  lastAverageComToAnkle = averageComToAnkle;
  bodyToCom += bodyToComOffset;

  Pose3D bodyToLeftAnkle(comToLeftAnkle.rotation, bodyToCom + comToLeftAnkle.translation);

  Pose3D bodyToRightAnkle(comToRightAnkle.rotation, bodyToCom + comToRightAnkle.translation);
  bool reachable = m_ik->calculateLegJoints(Pose2Matrix(bodyToLeftAnkle), Pose2Matrix(bodyToRightAnkle), joint_positions);

  for(int i = 0; i < 7; ++i)
  {
    if(reachable || this->bodyToCom == Vector3<>())
    {
      if(reachable)
      {
        this->bodyToCom = bodyToCom; // store the working bodyToCom offset
      }

      RobotModel robotModel(joint_positions, theMassCalibration);

      // TODO: improve this by not calculating the whole limb/mass model in each iteration

      Pose3D tmpComToLeftAnkle = Pose3D(-robotModel.centerOfMass).conc(robotModel.limbs[MassCalibration::footLeft]);
      Pose3D tmpComToRightAnkle = Pose3D(-robotModel.centerOfMass).conc(robotModel.limbs[MassCalibration::footRight]);
      tmpComToLeftAnkle.rotation = RotationMatrix();
      tmpComToRightAnkle.rotation = RotationMatrix();

      Vector3<> tmpAverageComToAnkle = (tmpComToLeftAnkle.translation + tmpComToRightAnkle.translation) * 0.5f;
      bodyToComOffset = (averageComToAnkle - tmpAverageComToAnkle) * 1.3f;
    }
    else
    {
      bodyToCom = this->bodyToCom; // recover last working bodyToCom offset
      bodyToComOffset *= 0.5f; // reduce last bodyToComOffset
    }

    bodyToCom += bodyToComOffset;

    bodyToLeftAnkle.translation = bodyToCom + comToLeftAnkle.translation;
    bodyToRightAnkle.translation = bodyToCom + comToRightAnkle.translation;

    reachable = m_ik->calculateLegJoints(Pose2Matrix(bodyToLeftAnkle), Pose2Matrix(bodyToRightAnkle), joint_positions);
    RobotModel robotModel(joint_positions, theMassCalibration);

    if(abs(bodyToComOffset.x) < 0.05 && abs(bodyToComOffset.y) < 0.05 && abs(bodyToComOffset.z) < 0.05)
    {
      break;
    }
  }

  std::vector<float>::iterator startIndex = joint_positions.begin()+2;
  std::vector<float>::iterator endIndex = startIndex + nu_nextLeftArmJoints.size();
  nu_nextLeftArmJoints.assign(startIndex, endIndex);
  startIndex = endIndex;
  endIndex = startIndex + nu_nextRightArmJoints.size();
  nu_nextRightArmJoints.assign(startIndex, endIndex);
  startIndex = endIndex;
  endIndex = startIndex + nu_nextLeftLegJoints.size();
  nu_nextLeftLegJoints.assign(startIndex, endIndex);
  startIndex = endIndex;
  endIndex = startIndex + nu_nextRightLegJoints.size();
  nu_nextRightLegJoints.assign(startIndex, endIndex);
  
  // Hack to move both feet.
  const unsigned int hip_yaw_index = 2;
  if( !kickPlayer.isActive()/*Here to test kick, remove if kick works*/){
      if (fabs(nu_nextRightLegJoints[hip_yaw_index]) < fabs(nu_nextLeftLegJoints[hip_yaw_index]))
      {
        nu_nextRightLegJoints[hip_yaw_index] = nu_nextLeftLegJoints[hip_yaw_index] / -2.0f;
        nu_nextLeftLegJoints[hip_yaw_index] /= 2.0f;
      }
      else if (fabs(nu_nextRightLegJoints[hip_yaw_index]) >  fabs(nu_nextLeftLegJoints[hip_yaw_index]))
      {
        nu_nextLeftLegJoints[hip_yaw_index] = nu_nextRightLegJoints[hip_yaw_index] / -2.0f;
        nu_nextRightLegJoints[hip_yaw_index] /= 2.0f;
      }
  }
  // If generated motion was a standing motion, save this as the new initial target.
  if(currentMotionType == stand)
  {
      m_initial_lleg.assign(nu_nextLeftLegJoints.begin(), nu_nextLeftLegJoints.end());
      m_initial_rleg.assign(nu_nextRightLegJoints.begin(), nu_nextRightLegJoints.end());
  }

    return;
}

void WalkingEngine::generateOutput(/*WalkingEngineOutput& walkingEngineOutput*/)
{
//  if(observedPendulumPlayer.isActive())
//  {
//    const float stepDuration = (observedPendulumPlayer.te - observedPendulumPlayer.next.tb) * 2.f;
//    walkingEngineOutput.speed.translation = Vector2<>(observedPendulumPlayer.s.translation.x + observedPendulumPlayer.next.s.translation.x, observedPendulumPlayer.s.translation.y + observedPendulumPlayer.next.s.translation.y) / stepDuration;
//    walkingEngineOutput.speed.rotation = (observedPendulumPlayer.s.rotation + observedPendulumPlayer.next.s.rotation) / stepDuration;
//  }
//  else
//    walkingEngineOutput.speed = Pose2D();

//  walkingEngineOutput.odometryOffset = odometryOffset;
//  walkingEngineOutput.upcomingOdometryOffset = upcomingOdometryOffset;
//  walkingEngineOutput.upcomingOdometryOffsetValid = upcomingOdometryOffsetValid;
//  walkingEngineOutput.isLeavingPossible = currentMotionType == stand;
//  if(currentMotionType == stepping)
//    walkingEngineOutput.positionInWalkCycle = 0.5f * ((observedPendulumPlayer.t - observedPendulumPlayer.tb) / (observedPendulumPlayer.te - observedPendulumPlayer.tb)) + (observedPendulumPlayer.supportLeg == left ? 0.5f : 0.f);
//  else
//    walkingEngineOutput.positionInWalkCycle = 0.f;
//  walkingEngineOutput.enforceStand = false;
//  walkingEngineOutput.instability = 0.f;
//  walkingEngineOutput.executedWalk = theMotionRequest.walkRequest;
//  walkingEngineOutput.executedWalk.kickType = kickPlayer.isActive() ? kickPlayer.getType() : KickPlayer::none;
//  (JointRequest&)walkingEngineOutput = jointRequest;

    float default_arm_stifness = 30.0f;
    // Set the arm positions.
    m_actions->add(NUActionatorsData::RArm, Platform->getTime(), nu_nextRightArmJoints, default_arm_stifness);
    m_actions->add(NUActionatorsData::LArm, Platform->getTime(), nu_nextLeftArmJoints, default_arm_stifness);

    float default_leg_stifness = 75.0f;

    // Reduce stiffness if we are standing.
    if(currentMotionType == stand)
    {
        default_leg_stifness = 75.0f;
    }

    // Set the leg positions.
    std::vector<float> legstiffness(m_actions->getSize(NUActionatorsData::LLeg), default_leg_stifness);
    legstiffness[5] = p.standHardnessAnkleRoll;
    legstiffness[6] = p.standHardnessAnklePitch;

    m_actions->add(NUActionatorsData::RLeg, 0, nu_nextRightLegJoints, legstiffness);
    m_actions->add(NUActionatorsData::LLeg, 0, nu_nextLeftLegJoints, legstiffness);
    return;
}

void WalkingEngine::generateDummyOutput(/*WalkingEngineOutput& walkingEngineOutput*/)
{
    return;
//  walkingEngineOutput.speed = Pose2D();
//  walkingEngineOutput.odometryOffset = Pose2D();
//  walkingEngineOutput.upcomingOdometryOffset = Pose2D();
//  walkingEngineOutput.upcomingOdometryOffsetValid = true;
//  walkingEngineOutput.isLeavingPossible = true;
//  walkingEngineOutput.positionInWalkCycle = 0.f;
//  walkingEngineOutput.enforceStand = false;
//  walkingEngineOutput.instability = 0.f;
//  walkingEngineOutput.executedWalk = WalkRequest();
  // leaving joint data untouched
}

void WalkingEngine::generateNextStepSize(SupportLeg nextSupportLeg, StepType lastStepType, KickPlayer::KickType last_kick_type, PendulumParameters& next)
{
  if(nextSupportLeg == lastNextSupportLeg)
    next = nextPendulumParameters;
  else
  {
    lastNextSupportLeg = nextSupportLeg;

    StepSize lastStepSize = next.s;

    const float sign = nextSupportLeg == right ? 1.f : -1.f;
    next.type = unknown;
    next.s = StepSize();
    next.l = Vector3<>(p.walkLiftOffset.x, p.walkLiftOffset.y * sign, p.walkLiftOffset.z);
    next.al = Vector3<>(p.walkAntiLiftOffset.x, p.walkAntiLiftOffset.y * sign, p.walkAntiLiftOffset.z);
    next.lRotation = Vector3<>();
    next.r = Vector2<>(p.walkRefX, p.walkRefY * (-sign));
    next.c = Vector2<>();
    next.k = p.walkK;
    next.te = p.te;
    next.tb = -p.te;
    next.kickType = KickPlayer::none;
    next.sXLimit.max = p.speedMax.translation.x * (1.1f * 0.5f);
    next.sXLimit.min = p.speedMaxBackwards * (-1.1f * 0.5f);
    next.rXLimit.max = next.r.x + p.walkRefXSoftLimit.max;
    next.rXLimit.min = next.r.x + p.walkRefXSoftLimit.min;
    next.rYLimit.max = p.walkRefY + p.walkRefYLimit.max;
    next.rYLimit.min = p.walkRefY + p.walkRefYLimit.min;

    switch(lastStepType)
    {
    case toStand:
      next.te = next.tb = 0.f;
//      if(theMotionRequest.motion == MotionRequest::bike)
//        next.r.x  = p.standBikeRefX;
//      else
      next.r.x = p.walkRefX;
      next.x0 = Vector2<>(0.f, -next.r.y);
      next.xv0 = Vector2<>();
      next.xtb = Vector2<>(next.r.x, 0.f);
      next.xvtb = next.xv0;
      break;
    case toStandLeft:
    case toStandRight:
      assert(false); // TODO!
      break;

    default:

      switch(requestedMotionType)
      {
      case stand:
//        if(theMotionRequest.motion == MotionRequest::bike && (nextSupportLeg == left) == theMotionRequest.bikeRequest.mirror)
//          break;
        if(abs(lastStepSize.translation.x) > p.speedMax.translation.x * 0.5f)
          break;
        next.type = toStand;
        next.te = p.te;
        next.tb = -p.te;
        break;

      case standLeft:
      case standRight:
        if((nextSupportLeg == left && requestedMotionType == standLeft) || (nextSupportLeg == right && requestedMotionType == standRight))
        {
          assert(false); // TODO!
          next.type = requestedMotionType == standLeft ? toStandLeft : toStandRight;
          //next.r = Vector2<>(0.f, (p.standComPosition.y - p.kickComPosition.y + p.kickX0Y) * (-sign));
          //next.x0 = Vector2<>(0.f, p.kickX0Y * sign);
          //next.k = p.kickK;
        }
        break;
      default:
        break;
      }
      if(next.type == unknown)
      {
        if(!instable &&  getKickType( m_ball_position, m_ball_target) != KickPlayer::none && !kickPlayer.isKickStandKick( getKickType( m_ball_position, m_ball_target)) &&
           kickPlayer.isKickMirrored( getKickType( m_ball_position, m_ball_target)) == (nextSupportLeg == left) &&
            getKickType( m_ball_position, m_ball_target) != lastExecutedWalkingKick)
        {
          lastExecutedWalkingKick =  getKickType( m_ball_position, m_ball_target);
          next.kickType =  getKickType( m_ball_position, m_ball_target);
          kickPlayer.getKickPreStepSize(next.kickType, next.s.rotation, next.s.translation);
          next.r.x = kickPlayer.getKickRefX(next.kickType, next.r.x);
          next.rXLimit.max = next.r.x + p.walkRefXSoftLimit.max;
          next.rXLimit.min = next.r.x + p.walkRefXSoftLimit.min;
          next.rYLimit.max = p.walkRefY + p.walkRefYLimitAtFullSpeedX.max;
          next.rYLimit.min = p.walkRefY + p.walkRefYLimitAtFullSpeedX.min;
          float duration = kickPlayer.getKickDuration(next.kickType);
          if(duration != 0.f)
          {
            next.te = duration * 0.25f;
            next.tb = -next.te;
          }
        }

        else if( last_kick_type != KickPlayer::none)
        {
          kickPlayer.getKickStepSize(last_kick_type, next.s.rotation, next.s.translation);
          next.r.x = kickPlayer.getKickRefX(last_kick_type, next.r.x);
          next.rXLimit.max = next.r.x + p.walkRefXSoftLimit.max;
          next.rXLimit.min = next.r.x + p.walkRefXSoftLimit.min;
          next.rYLimit.max = p.walkRefY + p.walkRefYLimitAtFullSpeedX.max;
          next.rYLimit.min = p.walkRefY + p.walkRefYLimitAtFullSpeedX.min;
          float duration = kickPlayer.getKickDuration(last_kick_type);
          if(duration != 0.f)
          {
            next.te = duration * 0.25f;
            next.tb = -next.te;
          }
        }

        else if(instable)
        {
          // nothing
        }
        else
        {
          if( getKickType( m_ball_position, m_ball_target) == KickPlayer::none)
            lastExecutedWalkingKick = KickPlayer::none;

          // get requested walk target and speed
          Pose2D walkTarget = requestedWalkTarget;
          //Pose2D requestedSpeed = theMotionRequest.walkRequest.speed;
          Pose2D requestedSpeed = Pose2D(m_speed_yaw, 10*m_speed_x, 10*m_speed_y);
//          if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode) // remove upcoming odometry offset
//          {

//            Pose2D upcomingOdometryOffset = observedPendulumPlayer.next.s - stepOffset * 0.5f; // == (observedPendulumPlayer.next.s - stepOffset) * 0.5f + observedPendulumPlayer.next.s * 0.5f
//            //upcomingOdometryOffset -= observedPendulumPlayer.s * 0.5f;

//            upcomingOdometryOffset.translation.x *= p.odometryUpcomingScale.translation.x;
//            upcomingOdometryOffset.translation.y *= p.odometryUpcomingScale.translation.y;
//            upcomingOdometryOffset.rotation *= p.odometryUpcomingScale.rotation;

//            float sign = observedPendulumPlayer.supportLeg == left ? -1.f : 1.f;
//            Pose2D up(p.odometryUpcomingOffset.rotation * sign, p.odometryUpcomingOffset.translation.x, p.odometryUpcomingOffset.translation.y * sign);
//            upcomingOdometryOffset += up;

//            walkTarget -= upcomingOdometryOffset;
//            walkTarget -= up;
//            requestedSpeed = Pose2D(walkTarget.rotation * 2.f / p.odometryUpcomingScale.rotation, walkTarget.translation.x * 2.f / p.odometryUpcomingScale.translation.x, walkTarget.translation.y * 2.f / p.odometryUpcomingScale.translation.y);

//            // x-speed clipping to handle limited deceleration
//            //float maxSpeedForTargetX = sqrt(2.f * abs(requestedSpeed.translation.x) * p.speedMaxChangeX);
//            //if(abs(requestedSpeed.translation.x) > maxSpeedForTargetX)
//            //requestedSpeed.translation.x = requestedSpeed.translation.x >= 0.f ? maxSpeedForTargetX : -maxSpeedForTargetX;
//          }
//          else if(theMotionRequest.walkRequest.mode == WalkRequest::percentageSpeedMode)
//          {
//            requestedSpeed.rotation *= p.speedMax.rotation;
//            requestedSpeed.translation.x *= (requestedSpeed.translation.x >= 0.f ? p.speedMax.translation.x : p.speedMaxBackwards);
//            requestedSpeed.translation.y *= p.speedMax.translation.y;
//          }

          // compute max speeds for the requested walk direction
          Pose2D maxSpeed(p.speedMax.rotation, requestedSpeed.translation.x < 0.f ? p.speedMaxBackwards : p.speedMax.translation.x, p.speedMax.translation.y);
          Vector3<> tmpSpeed(
            requestedSpeed.translation.x / (p.speedMaxMin.translation.x + maxSpeed.translation.x),
            requestedSpeed.translation.y / (p.speedMaxMin.translation.y + maxSpeed.translation.y),
            requestedSpeed.rotation / (p.speedMaxMin.rotation + maxSpeed.rotation));
          const float tmpSpeedAbs = tmpSpeed.abs();
          if(tmpSpeedAbs > 1.f)
          {
            tmpSpeed /= tmpSpeedAbs;
            tmpSpeed.x *= (p.speedMaxMin.translation.x + maxSpeed.translation.x);
            tmpSpeed.y *= (p.speedMaxMin.translation.y + maxSpeed.translation.y);
            tmpSpeed.z *= (p.speedMaxMin.rotation + maxSpeed.rotation);
            maxSpeed.translation.x = min(abs(tmpSpeed.x), maxSpeed.translation.x);
            maxSpeed.translation.y = min(abs(tmpSpeed.y), maxSpeed.translation.y);
            maxSpeed.rotation = min(abs(tmpSpeed.z), maxSpeed.rotation);
          }

//          // x-speed clipping to handle limited deceleration
//          if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
//          {
//            float maxSpeedForTargetX = sqrt(2.f * abs(requestedSpeed.translation.x) * p.speedMaxChange.translation.x);
//            if(abs(requestedSpeed.translation.x) > maxSpeedForTargetX)
//              requestedSpeed.translation.x = requestedSpeed.translation.x >= 0.f ? maxSpeedForTargetX : -maxSpeedForTargetX;

//            float maxSpeedForTargetY = sqrt(2.f * abs(requestedSpeed.translation.y) * p.speedMaxChange.translation.y);
//            if(abs(requestedSpeed.translation.y) > maxSpeedForTargetY)
//              requestedSpeed.translation.y = requestedSpeed.translation.y >= 0.f ? maxSpeedForTargetY : -maxSpeedForTargetY;

//            float maxSpeedForTargetR = sqrt(2.f * abs(requestedSpeed.rotation) * p.speedMaxChange.rotation);
//            if(abs(requestedSpeed.rotation) > maxSpeedForTargetR)
//              requestedSpeed.rotation = requestedSpeed.rotation >= 0.f ? maxSpeedForTargetR : -maxSpeedForTargetR;
//          }

          // max speed change clipping (y-only)
          // just clip y and r since x will be clipped by min/maxRX in computeRefZMP
          requestedSpeed.translation.y = Range<>(lastSelectedSpeed.translation.y - p.speedMaxChange.translation.y, lastSelectedSpeed.translation.y + p.speedMaxChange.translation.y).limit(requestedSpeed.translation.y);
          requestedSpeed.rotation = Range<>(lastSelectedSpeed.rotation - p.speedMaxChange.rotation, lastSelectedSpeed.rotation + p.speedMaxChange.rotation).limit(requestedSpeed.rotation);

          // clip requested walk speed to the computed max speeds
          if(abs(requestedSpeed.rotation) > maxSpeed.rotation)
            requestedSpeed.rotation = requestedSpeed.rotation > 0.f ? maxSpeed.rotation : -maxSpeed.rotation;
          if(abs(requestedSpeed.translation.x) > maxSpeed.translation.x)
            requestedSpeed.translation.x = requestedSpeed.translation.x > 0.f ? maxSpeed.translation.x : -maxSpeed.translation.x;
          if(abs(requestedSpeed.translation.y) > maxSpeed.translation.y)
            requestedSpeed.translation.y = requestedSpeed.translation.y > 0.f ? maxSpeed.translation.y : -maxSpeed.translation.y;

//          // clip requested walk speed to a target walk speed limit
//          if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
//          {
//            requestedSpeed.translation.x = Range<>(-p.speedMax.translation.x * theMotionRequest.walkRequest.speed.translation.x, p.speedMax.translation.x * theMotionRequest.walkRequest.speed.translation.x).limit(requestedSpeed.translation.x);
//            requestedSpeed.translation.y = Range<>(-p.speedMax.translation.y * theMotionRequest.walkRequest.speed.translation.y, p.speedMax.translation.y * theMotionRequest.walkRequest.speed.translation.y).limit(requestedSpeed.translation.y);
//            requestedSpeed.rotation = Range<>(-p.speedMax.rotation * theMotionRequest.walkRequest.speed.rotation, p.speedMax.rotation * theMotionRequest.walkRequest.speed.rotation).limit(requestedSpeed.rotation);
//          }

          // generate step size from requested walk speed
          next.s = StepSize(requestedSpeed.rotation, requestedSpeed.translation.x * 0.5f, requestedSpeed.translation.y);

          // adjust step duration according to the actual desired step size
          {
            // do this before the "just move the outer foot" clipping
            const float accClippedSpeedX = Range<>(lastSelectedSpeed.translation.x - p.speedMaxChange.translation.x, lastSelectedSpeed.translation.x + p.speedMaxChange.translation.x).limit(requestedSpeed.translation.x);
            const float accClippedStepSizeX = accClippedSpeedX * 0.5f;

            {
              const float xxSpeedFactor = (p.teAtFullSpeedX - p.te) / (p.speedMax.translation.x * 0.5f);
              const float yySpeedFactor = (p.teAtFullSpeedY - p.te) / p.speedMax.translation.y;
              next.te += abs(next.s.translation.y) * yySpeedFactor;
              next.te += abs(accClippedStepSizeX) * xxSpeedFactor;
              next.tb = -next.te;
            }

            {
              float xSpeedFactor = (p.walkRefXAtFullSpeedX -  p.walkRefX) / (p.speedMax.translation.x * 0.5f);
              next.r.x += abs(accClippedStepSizeX) * xSpeedFactor;
              next.rXLimit.max = next.r.x + p.walkRefXSoftLimit.max;
              next.rXLimit.min = next.r.x + p.walkRefXSoftLimit.min;
            }

            {
              float walkRefYLimitMax = p.walkRefYLimit.max;
              float walkRefYLimitMin = p.walkRefYLimit.min;
              {
                float xSpeedFactor = (p.walkRefYLimitAtFullSpeedX.max -  p.walkRefYLimit.max) / (p.speedMax.translation.x * 0.5f);
                walkRefYLimitMax += abs(accClippedStepSizeX) * xSpeedFactor;
              }
              {
                float xSpeedFactor = (p.walkRefYLimitAtFullSpeedX.min -  p.walkRefYLimit.min) / (p.speedMax.translation.x * 0.5f);
                walkRefYLimitMin += abs(accClippedStepSizeX) * xSpeedFactor;
              }

              float ySpeedFactor = (p.walkRefYAtFullSpeedY -  p.walkRefY) / p.speedMax.translation.y;
              float xSpeedFactor = (p.walkRefYAtFullSpeedX -  p.walkRefY) / (p.speedMax.translation.x * 0.5f);
              next.r.y += (abs(requestedSpeed.translation.y) * ySpeedFactor + abs(accClippedStepSizeX) * xSpeedFactor) * (-sign);
              next.rYLimit.max = abs(next.r.y) + walkRefYLimitMax;
              next.rYLimit.min = abs(next.r.y) + walkRefYLimitMin;
            }
          }

          // just move the outer foot, when walking sidewards or when rotating
          if((next.s.translation.y < 0.f && nextSupportLeg == left) || (next.s.translation.y > 0.f && nextSupportLeg != left))
            next.s.translation.y = 0.f;
          if((next.s.rotation < 0.f && nextSupportLeg == left) || (next.s.rotation > 0.f && nextSupportLeg != left))
            next.s.rotation = 0.f;
//          if((next.s.translation.y < 0.f && nextSupportLeg != left) || (next.s.translation.y > 0.f && nextSupportLeg == left))
//            next.s.translation.y = 0.f;
//          if((next.s.rotation < 0.f && nextSupportLeg != left) || (next.s.rotation > 0.f && nextSupportLeg == left))
//            next.s.rotation = 0.f;

          // clip to walk target
//          if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
//          {
//            if((next.s.translation.x > 0.f && walkTarget.translation.x > 0.f && next.s.translation.x * p.odometryUpcomingScale.translation.x > walkTarget.translation.x) || (next.s.translation.x < 0.f && walkTarget.translation.x < 0.f && next.s.translation.x * p.odometryUpcomingScale.translation.x < walkTarget.translation.x))
//              next.s.translation.x = walkTarget.translation.x / p.odometryUpcomingScale.translation.x;
//            if((next.s.translation.y > 0.f && walkTarget.translation.y > 0.f && next.s.translation.y * p.odometryUpcomingScale.translation.y > walkTarget.translation.y) || (next.s.translation.y < 0.f && walkTarget.translation.y < 0.f && next.s.translation.y * p.odometryUpcomingScale.translation.y < walkTarget.translation.y))
//              next.s.translation.y = walkTarget.translation.y / p.odometryUpcomingScale.translation.y;
//            if((next.s.rotation > 0.f && walkTarget.rotation > 0.f && next.s.rotation * p.odometryUpcomingScale.rotation > walkTarget.rotation) || (next.s.rotation < 0.f && walkTarget.rotation < 0.f && next.s.rotation * p.odometryUpcomingScale.rotation < walkTarget.rotation))
//              next.s.rotation = walkTarget.rotation / p.odometryUpcomingScale.rotation;
//          }
        }

        next.lRotation = Vector3<>(
                           p.walkLiftRotation.x * sign * fabs(next.s.translation.y) / p.speedMax.translation.y,
                           next.s.translation.x > 0.f ? (p.walkLiftRotation.y * next.s.translation.x / (p.speedMax.translation.x * 0.5f)) : 0,
                           p.walkLiftRotation.z * sign);

        {
          float xSpeedFactor = (p.walkLiftOffsetAtFullSpeedY.x - p.walkLiftOffset.x) / p.speedMax.translation.y;
          float ySpeedFactor = (p.walkLiftOffsetAtFullSpeedY.y - p.walkLiftOffset.y) / p.speedMax.translation.y;
          float zSpeedFactor = (p.walkLiftOffsetAtFullSpeedY.z - p.walkLiftOffset.z) / p.speedMax.translation.y;
          next.l.x += abs(next.s.translation.y) * xSpeedFactor;
          next.l.y += abs(next.s.translation.y) * ySpeedFactor * sign;
          next.l.z += abs(next.s.translation.y) * zSpeedFactor;
        }

        {
          float xSpeedFactor = (p.walkAntiLiftOffsetAtFullSpeedY.x - p.walkAntiLiftOffset.x) / p.speedMax.translation.y;
          float ySpeedFactor = (p.walkAntiLiftOffsetAtFullSpeedY.y - p.walkAntiLiftOffset.y) / p.speedMax.translation.y;
          float zSpeedFactor = (p.walkAntiLiftOffsetAtFullSpeedY.z - p.walkAntiLiftOffset.z) / p.speedMax.translation.y;
          next.al.x += abs(next.s.translation.y) * xSpeedFactor;
          next.al.y += abs(next.s.translation.y) * ySpeedFactor * sign;
          next.al.z += abs(next.s.translation.y) * zSpeedFactor;
        }
      }

      lastSelectedSpeed = Pose2D(lastStepSize.rotation + next.s.rotation, lastStepSize.translation.x + next.s.translation.x, lastStepSize.translation.y + next.s.translation.y);

      // next.r.y + next.x0.y * cosh(next.k * next.tb) = 0.f
      // => next.x0.y = - next.r.y / cosh(next.k * next.tb)
      next.x0 = Vector2<>(0.f, -next.r.y / cosh(next.k.y * next.tb));

      // next.xv0.x * sinh(next.k * next.tb) / next.k = next.s.translation.x * -0.5f
      // => next.xv0.x = next.s.translation.x * -0.5f * next.k / sinh(next.k * next.tb)
      next.xv0 = Vector2<>(next.s.translation.x * -0.5f * next.k.x / sinh(next.k.x * next.tb), 0.f);

      // next.r.y + next.x0.y * cosh(next.k * next.tb) = next.s.translation.y * -0.5f
      // => next.tb = -acosh((next.s.translation.y * -0.5f - next.r.y) / next.x0.y) / next.k
      next.tb = -saveAcosh((next.s.translation.y * -0.5f - next.r.y) / next.x0.y) / next.k.y;

      // next.r.x + next.xv0.x * sinh(next.k * next.tb) / k  = next.xtb.x
      next.xtb = Vector2<>(next.r.x + next.xv0.x * sinh(next.k.x * next.tb) / next.k.x, next.s.translation.y * -0.5f);

      // next.xvtb.x = next.xv0.x * cosh(next.k * next.tb)
      // next.xvtb.y = next.x0.y * next.k * sinh(next.k * next.tb)
      next.xvtb = Vector2<>(next.xv0.x * cosh(next.k.x * next.tb), next.x0.y * next.k.y * sinh(next.k.y * next.tb));

      next.originalRX = next.r.x;
    }
    nextPendulumParameters = next;
  }
}

void WalkingEngine::computeOdometryOffset()
{
//  if(p.odometryUseTorsoMatrix)
//  {
//    // "measured" odometry
//    if(lastTorsoMatrix.translation.z != 0.)
//    {
//      Pose3D odometryOffset3D(lastTorsoMatrix);
//      odometryOffset3D.conc(theTorsoMatrix.offset);
//      odometryOffset3D.conc(theTorsoMatrix.invert());
//      odometryOffset.translation.x = odometryOffset3D.translation.x;
//      odometryOffset.translation.y = odometryOffset3D.translation.y;
//      odometryOffset.rotation = odometryOffset3D.rotation.getZAngle();
//    }
//    lastTorsoMatrix = theTorsoMatrix;
//  }
//  else
//  {
//    // calculated odometry
//    if(observedPendulumPlayer.supportLeg == lastSupportLeg)
//      odometryOffset = (stepOffset - lastStepOffset) * 0.5f;
//    else
//      odometryOffset = (stepOffset + observedPendulumPlayer.s * 2.f - lastStepOffset) * 0.5f; // == ((observedPendulumPlayer.s - lastStepOffset) + (stepOffset - (observedPendulumPlayer.s * -1f))) * 0.5f;
//  }

//  upcomingOdometryOffset = observedPendulumPlayer.next.s - stepOffset * 0.5f; // == (observedPendulumPlayer.next.s - stepOffset) * 0.5f + observedPendulumPlayer.next.s * 0.5f

//  // HACK: somehow this improves the accuracy of the upcoming odometry offset for target walks (but i have no idea why)
//  upcomingOdometryOffset -= (observedPendulumPlayer.s + observedPendulumPlayer.next.s) * 0.5f;

//  float sign = observedPendulumPlayer.supportLeg == left ? -1.f : 1.f;
//  Pose2D up(p.odometryUpcomingOffset.rotation * sign, p.odometryUpcomingOffset.translation.x, p.odometryUpcomingOffset.translation.y * sign);
//  upcomingOdometryOffset += up;
//  upcomingOdometryOffsetValid = observedPendulumPlayer.supportLeg == pendulumPlayer.supportLeg;
//  if(!upcomingOdometryOffsetValid)
//    upcomingOdometryOffset += pendulumPlayer.next.s;
//  else
//    upcomingOdometryOffsetValid = (observedPendulumPlayer.te - observedPendulumPlayer.t) > 0.040f;

//  lastSupportLeg = observedPendulumPlayer.supportLeg;
//  lastStepOffset = stepOffset;

//  odometryOffset.translation.x *= p.odometryScale.translation.x;
//  odometryOffset.translation.y *= p.odometryScale.translation.y;
//  odometryOffset.rotation *= p.odometryScale.rotation;

//  upcomingOdometryOffset.translation.x *= p.odometryUpcomingScale.translation.x;
//  upcomingOdometryOffset.translation.y *= p.odometryUpcomingScale.translation.y;
//  upcomingOdometryOffset.rotation *= p.odometryUpcomingScale.rotation;

//  if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
//    requestedWalkTarget -= odometryOffset;
}

Matrix WalkingEngine::Pose2Matrix(const Pose3D& pose)
{
    Matrix result(4,4,true);
    result[0][0] = pose.rotation.c0.x;
    result[1][0] = pose.rotation.c0.y;
    result[2][0] = pose.rotation.c0.z;

    result[0][1] = pose.rotation.c1.x;
    result[1][1] = pose.rotation.c1.y;
    result[2][1] = pose.rotation.c1.z;

    result[0][2] = pose.rotation.c2.x;
    result[1][2] = pose.rotation.c2.y;
    result[2][2] = pose.rotation.c2.z;

    result[0][3] = pose.translation.x;
    result[1][3] = pose.translation.y;
    result[2][3] = pose.translation.z;

    return result;
}
