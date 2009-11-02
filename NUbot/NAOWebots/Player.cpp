#include "Player.hpp"
#include "NaoCam.hpp"
#include "InfoMessage.hpp"
#include "nao_soccer_supervisor/RoboCupGameControlData.h"
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Servo.hpp>
#include <webots/LED.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/GPS.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Servo.hpp>
#include <webots/utils/Motion.hpp>
#include <string>
#include <cstring>
#include <cmath>
#include <iostream>

using namespace std;
using namespace webots;

const int Player::SIMULATION_STEP = 40;  // milliseconds
const int CAMERA_STEP = 160;  // camera refresh rate in milliseconds
const double CAMERA_OFFSET_ANGLE = 0.6981;  // 40 degrees between cameras axes

Player::Player(int playerID) {
 
  // need space to store incoming data
  gameControlData = new RoboCupGameControlData;

  this->playerID = playerID;

  if (getName().find("red") != string::npos)
    team = TEAM_RED;
  else
    team = TEAM_BLUE;

  // initialize accelerometer
  accelerometer = getAccelerometer("accelerometer");
  accelerometer->enable(SIMULATION_STEP);

  // initialize gyro
  gyro = getGyro("gyro");
  //gyro.enable(SIMULATION_STEP);   // uncomment only if needed !

  // get "HeadYaw" and "HeadPitch" motors and enable position feedback
  headYaw = getServo("HeadYaw");
  headYaw->enablePosition(SIMULATION_STEP);
  headPitch = getServo("HeadPitch");
  headPitch->enablePosition(SIMULATION_STEP);

  // get all LEDs
  chestLed = getLED("ChestBoard/Led");
  rightEyeLed = getLED("Face/Led/Right");
  leftEyeLed = getLED("Face/Led/Left");
  rightEarLed = getLED("Ears/Led/Right");
  leftEarLed = getLED("Ears/Led/Left");
  rightFootLed = getLED("RFoot/Led");
  leftFootLed = getLED("LFoot/Led");

  // make eyes shine blue
  rightEyeLed->set(0x2222ff);
  leftEyeLed->set(0x2222ff);

  // get camera
  camera = (NaoCam*)getCamera("camera");
  camera->enable(CAMERA_STEP);

  // foot sole touch sensors
  const string TOUCH_SENSOR_NAMES[NUM_TOUCH_SENSORS] = {
    "RFsrFL", "RFsrFR", "RFsrBR", "RFsrBL",
    "LFsrFL", "LFsrFR", "LFsrBR", "LFsrBL"
  };

  for (int i = 0; i < NUM_TOUCH_SENSORS; i++) {
    fsr[i] = getTouchSensor(TOUCH_SENSOR_NAMES[i]);
    //fsr[i]->enable(SIMULATION_STEP);  // uncomment only if needed !
  }

  // emitter/receiver devices that can be used for inter-robot communication
  // and for receiving RobotCupGameControleData
  emitter = getEmitter("emitter");
  receiver = getReceiver("receiver");
  receiver->enable(SIMULATION_STEP);

  // for sending 'move' request to Supervisor
  superEmitter = getEmitter("super_emitter");

  // useful to know the position of the robot
  // the real Nao does not have a GPS, this is for testing only
  // therefore the GPS info will be blurred during Robotstadium contest matches
  gps = getGPS("gps");
  //gps->enable(SIMULATION_STEP);  // uncomment only if needed !

  // initialize ultrasound sensors
  topLeftUltrasound = getDistanceSensor("US/TopLeft");
  topRightUltrasound = getDistanceSensor("US/TopRight");
  bottomLeftUltrasound = getDistanceSensor("US/BottomLeft");
  bottomRightUltrasound = getDistanceSensor("US/BottomRight");
  //topLeftUltrasound->enable(SIMULATION_STEP);  // uncomment only if needed !
  //topRightUltrasound->enable(SIMULATION_STEP);  // uncomment only if needed !
  //bottomLeftUltrasound->enable(SIMULATION_STEP);  // uncomment only if needed !
  //bottomRightUltrasound->enable(SIMULATION_STEP);  // uncomment only if needed !

  // load motion
  standUpFromFrontMotion = new Motion("../motions/StandUpFromFront.motion");
}

Player::~Player() {
  delete standUpFromFrontMotion;
  delete gameControlData;
}

// overriden to create NaoCam instead of Camera
Camera *Player::createCamera(const string &name) const {

  // choose goal color according to my team color
  if (team == TEAM_RED)
    return new NaoCam(name, NaoCam::SKY_BLUE, (Robot*)this);
  else
    return new NaoCam(name, NaoCam::YELLOW, (Robot*)this);
}

// play until the specified motion is over
void Player::playMotion(Motion *motion) {

  // don't move if the game is not playing
  if (gameControlData->state != STATE_PLAYING) {
    return;
  }

  // play to the end
  motion->play();
  do {
    runStep();
  }
  while (! motion->isOver());
}

// vertical acceleration should be approx earth acceleration (9.81 m/s^2). If a much lower value
// is measured, this probably indicates that the robot is no longer in an upright position.
// The x-axis is oriented towards the front, the y-axis towards the right hand, the z-axis looks
// towards the head.
void Player::getUpIfNecessary() {
  const double *acc = accelerometer->getValues();
  if (acc[2] < 5.0)
    playMotion(standUpFromFrontMotion);
}

// move head from left to right and from right to left
// until the ball is sighted or the scan motion is over
void Player::headScan() {
  const int STEPS = 30;
  const double HEAD_YAW_MAX = 2.0;

  headPitch->setPosition(0.0);  // horizontal head
  camera->selectTop();  // use top camera

  // left to right using TOP camera
  for (int i = 0; i < STEPS; i++) {
    double yawAngle = ((double)i / (STEPS - 1) * 2.0 - 1.0) * HEAD_YAW_MAX;
    headYaw->setPosition(yawAngle);
    step(SIMULATION_STEP);
    camera->processImage();
    if (camera->getBallDirectionAngle() != NaoCam::UNKNOWN)
      return;
  }

  camera->selectBottom();  // use bottom camera

  // right to left using BOTTOM camera
  for (int i = STEPS - 1; i >= 0; i--) {
    double yawAngle = ((double)i / (STEPS - 1) * 2.0 - 1.0) * HEAD_YAW_MAX;
    headYaw->setPosition(yawAngle);
    step(SIMULATION_STEP);
    camera->processImage();
    if (camera->getBallDirectionAngle() != NaoCam::UNKNOWN)
      return;
  }

  // ball was not found: restore head straight position
  headYaw->setPosition(0.0);
}

double Player::getBallDirection() const {
  if (camera->getBallDirectionAngle() == NaoCam::UNKNOWN)
    return NaoCam::UNKNOWN;
  else
    return camera->getBallDirectionAngle() - headYaw->getPosition();
}

// compute floor distance between robot (feet) and ball
// 0.51 -> approx robot camera base height with respect to ground in a standard posture of the robot
// 0.043 -> ball radius
double Player::getBallDistance() const {
  if (camera->getBallElevationAngle() == NaoCam::UNKNOWN)
    return NaoCam::UNKNOWN;

  double ballElev = camera->getBallElevationAngle() - headPitch->getPosition() - camera->getOffsetAngle();
  return (0.51 - 0.043) / tan(-ballElev);
}

// turn head towards ball if ball position is known
void Player::trackBall() {
  const double P = 0.7;
  double ballDirection = camera->getBallDirectionAngle();
  double ballElevation = camera->getBallElevationAngle();

  if (ballDirection == NaoCam::UNKNOWN)
    return;

  // compute target head pitch
  double pitch = headPitch->getPosition() - ballElevation * P;

  if (pitch < -0.4 && ! camera->isTopSelected()) { // need to switch to TOP camera ?
    //cout << "switched to TOP camera\n";
    camera->selectTop();
    pitch += NaoCam::OFFSET_ANGLE;  // move head down 40 degrees
    headPitch->setPosition(pitch);
    sleepSteps(8);  // allow some time to move head
    camera->processImage();
  }
  else if (pitch > 0.5 && camera->isTopSelected()) { // need to switch to BOTTOM camera ?
    //System.out.println("switched to BOTTOM camera");
    camera->selectBottom();
    pitch -= NaoCam::OFFSET_ANGLE;  // move head up 40 degrees
    headPitch->setPosition(pitch);
    sleepSteps(8);  // allow some time to move head
    camera->processImage();
  }

  headPitch->setPosition(pitch);
  headYaw->setPosition(headYaw->getPosition() - ballDirection * P);
}

// update torso LED color according to game state
void Player::updateChestLed() {
  switch (gameControlData->state) {
    case STATE_INITIAL:
    case STATE_FINISHED:
      chestLed->set(0x000000);  // off
      break;
    case STATE_READY:
      chestLed->set(0x2222ff);  // blue
      break;
    case STATE_SET:
      chestLed->set(0xffff22);  // yellow
      break;
    case STATE_PLAYING:
      chestLed->set(0x22ff22);  // green
      break;
  }
}

void Player::readIncomingMessages() {
  while (receiver->getQueueLength() > 0) {
    const void *data = receiver->getData();

    // very important: verify packet header because all kind of packets may be flying around during a match
    if (memcmp(data, GAMECONTROLLER_STRUCT_HEADER, sizeof(GAMECONTROLLER_STRUCT_HEADER) - 1) == 0) {
      memcpy(gameControlData, data, sizeof(RoboCupGameControlData));
      updateChestLed();
    }
    else if (memcmp(data, INFO_MESSAGE_STRUCT_HEADER, sizeof(INFO_MESSAGE_STRUCT_HEADER) - 1) == 0) {
      struct InfoMessage info;
      memcpy(&info, data, sizeof(InfoMessage));
      cerr << "player" << info.playerID << " is " << info.distanceToBall << "meters from the ball" << endl;
    }
    // unidentified messages will be silently ignored, otherwise: uncomment these two lines
    // else
    //   cerr << "readIncomingMessages(): received unexpected message of " << receiver->getDataSize() << " bytes" << endl;

    receiver->nextPacket();
  }
}

// example on how to send an info message to teammates
// these messages are not sent to the Supervisor
void Player::sendInfoMessage() {
  InfoMessage info;
  memcpy(info.header, INFO_MESSAGE_STRUCT_HEADER, sizeof(GAMECONTROLLER_STRUCT_HEADER) - 1);
  info.distanceToBall = getBallDistance();
  // info.blalba = your stuff ...
  emitter->send(&info, sizeof(InfoMessage));
}

// move the robot to a specified position (via a message sent to the Supervisor)
// [tx ty tz]: the new robot position, alpha: the robot's heading direction
// For debugging only: this is disabled during the contest rounds
void Player::sendMoveRobotMessage(double tx, double ty, double tz, double alpha) {
  char buf[256];
  sprintf(buf, "move robot %d %d %f %f %f %f", team + 1 % 2, playerID, tx, ty, tz, alpha);
  superEmitter->send(buf, strlen(buf) + 1);
}

// move the ball to a specified position (via a message sent to the Supervisor)
// [tx ty tz]: the new ball position
// For debugging only: this is disabled during the contest rounds
void Player::sendMoveBallMessage(double tx, double ty, double tz) {
  char buf[256];
  sprintf(buf, "move ball %f %f %f", tx, ty, tz);
  superEmitter->send(buf, strlen(buf) + 1);
}

void Player::runStep()  {
  trackBall();
  step(SIMULATION_STEP);
  camera->processImage();
  readIncomingMessages();
}

void Player::sleepSteps(int steps) {
  for (int i = 0; i < steps; i++) {
    step(SIMULATION_STEP);
    readIncomingMessages();
  }
}
