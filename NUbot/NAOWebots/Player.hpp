#ifndef PLAYER_HPP
#define PLAYER_HPP

//-----------------------------------------------------------------------------
//  File:         Player class (to be used in a Webots controllers)
//  Description:  Base class for FieldPlayer and GoalKeeper
//  Project:      Robotstadium, the online robot soccer competition
//  Author:       Yvan Bourquin - www.cyberbotics.com
//  Date:         May 4, 2008
//  Changes:      
//-----------------------------------------------------------------------------

#include <webots/Robot.hpp>

namespace webots {
  class Accelerometer;
  class Gyro;
  class DistanceSensor;
  class LED;
  class TouchSensor;
  class Emitter;
  class Receiver;
  class GPS;
  class Motion;
  class Servo;
}

class NaoCam;

using namespace webots;
using namespace std;

class Player : public Robot {
public:
  Player(int playerID);
  virtual ~Player();

  enum { NUM_TOUCH_SENSORS = 8 };

  // pure virtual: effective implementation in derived classes
  virtual void run() = 0;

protected:
  virtual void runStep();
  void playMotion(Motion *motion);
  void getUpIfNecessary();
  double getBallDirection() const;
  double getBallDistance() const;
  void headScan();

  // global control step (must be a multiple of WorldInfo.basicTimeStep)
  static const int SIMULATION_STEP;

  struct RoboCupGameControlData *gameControlData;
  int playerID;
  int team;  // TEAM_RED or TEAM_BLUE

  // devices
  NaoCam *camera;
  Servo *headYaw, *headPitch;
  Accelerometer *accelerometer;
  Gyro *gyro;
  DistanceSensor *topLeftUltrasound, *topRightUltrasound, *bottomLeftUltrasound, *bottomRightUltrasound;
  LED *chestLed, *rightEyeLed, *leftEyeLed, *rightEarLed, *leftEarLed, *rightFootLed, *leftFootLed;
  TouchSensor *fsr[NUM_TOUCH_SENSORS];  // force sensitive resistors
  Emitter *emitter, *superEmitter;
  Receiver *receiver;
  GPS *gps;  // for debugging only ! This device does not exist on the real robot.
  Motion *standUpFromFrontMotion;

private:
  Camera *createCamera(const string&) const;
  void trackBall();
  void updateChestLed();
  void readIncomingMessages();
  void sendInfoMessage();
  void sendMoveRobotMessage(double tx, double ty, double tz, double alpha);
  void sendMoveBallMessage(double tx, double ty, double tz);
  void sleepSteps(int steps);
};

#endif
