#include "FieldPlayer.hpp"
#include "NaoCam.hpp"
#include <webots/utils/Motion.hpp>
#include <webots/Servo.hpp>
#include <cmath>

using namespace webots;

FieldPlayer::FieldPlayer(int playerID) : Player(playerID) {
  goalDir = 0.0; // interpolated goal direction (with respect to front direction of robot body)
  backwardsMotion = new Motion("../motions/Backwards.motion");
  forwardsMotion = new Motion("../motions/Forwards.motion");
  forwards50Motion = new Motion("../motions/Forwards50.motion");
  turnRight40Motion = new Motion("../motions/TurnRight40.motion");
  turnLeft40Motion = new Motion("../motions/TurnLeft40.motion");
  turnRight60Motion = new Motion("../motions/TurnRight60.motion");
  turnLeft60Motion = new Motion("../motions/TurnLeft60.motion");
  turnLeft180Motion = new Motion("../motions/TurnLeft180.motion");
  sideStepRightMotion = new Motion("../motions/SideStepRight.motion");
  sideStepLeftMotion = new Motion("../motions/SideStepLeft.motion");

  // move arms down
  getServo("LShoulderPitch")->setPosition(1.5);
  getServo("RShoulderPitch")->setPosition(1.5);
}

FieldPlayer::~FieldPlayer() {
  delete backwardsMotion;
  delete forwardsMotion;
  delete forwards50Motion;
  delete turnRight40Motion;
  delete turnLeft40Motion;
  delete turnRight60Motion;
  delete turnLeft60Motion;
  delete turnLeft180Motion;
  delete sideStepRightMotion;
  delete sideStepLeftMotion;
}

// normalize angle between -pi and +pi
static double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// relative body turn
void FieldPlayer::turnBodyRel(double angle) {
  if (angle > 0.7)
    turnRight60();
  else if (angle < -0.7)
    turnLeft60();
  else if (angle > 0.3)
    turnRight40();
  else if (angle < -0.3)
    turnLeft40();
}

void FieldPlayer::runStep() {
  Player::runStep();
  double dir = camera->getGoalDirectionAngle();
  if (dir != NaoCam::UNKNOWN)
    goalDir = dir - headYaw->getPosition();
}

void FieldPlayer::turnRight60() {
  playMotion(turnRight60Motion); // 59.2 degrees
  goalDir = normalizeAngle(goalDir - 1.033);
}

void FieldPlayer::turnLeft60() {
  playMotion(turnLeft60Motion); // 59.2 degrees
  goalDir = normalizeAngle(goalDir + 1.033);
}

void FieldPlayer::turnRight40() {
  playMotion(turnRight40Motion); // 39.7 degrees
  goalDir = normalizeAngle(goalDir - 0.693);
}

void FieldPlayer::turnLeft40() {
  playMotion(turnLeft40Motion); // 39.7 degrees
  goalDir = normalizeAngle(goalDir + 0.693);
}

void FieldPlayer::turnLeft180() {
  playMotion(turnLeft180Motion); // 163.6 degrees
  goalDir = normalizeAngle(goalDir + 2.855);
}

void FieldPlayer::run() {
  step(SIMULATION_STEP);

  while (true) {
    runStep();
    getUpIfNecessary();

    while (getBallDirection() == NaoCam::UNKNOWN) {
      //cout << "searching the ball" << endl;
      getUpIfNecessary();
      if (getBallDirection() != NaoCam::UNKNOWN) break;
      headScan();
      if (getBallDirection() != NaoCam::UNKNOWN) break;
      playMotion(backwardsMotion);
      if (getBallDirection() != NaoCam::UNKNOWN) break;
      headScan();
      if (getBallDirection() != NaoCam::UNKNOWN) break;
      turnLeft180();
    }

    double ballDir = getBallDirection();
    double ballDist = getBallDistance();

    //cout << "ball dist: " << ballDist << " ball dir: " << ballDir << " goal dir: " << goalDir << endl;

    if (ballDist < 0.3) {
      //cout << "short distance" << endl;

      if (ballDir < -0.15)
        playMotion(sideStepLeftMotion);
      else if (ballDir > 0.15)
        playMotion(sideStepRightMotion);
      else if (goalDir < -0.35)
        turnLeft40();
      else if (goalDir > 0.35)
        turnRight40();
      else {
        //cout << "shooting !!!" << endl;
        playMotion(forwards50Motion);
      }
    }
    else {
      //cout << "long distance" << endl;
      double goDir = normalizeAngle(ballDir - goalDir);

      if (goDir < ballDir - 0.5)
        goDir = ballDir - 0.5;
      else if (goDir > ballDir + 0.5)
        goDir = ballDir + 0.5;

      goDir = normalizeAngle(goDir);

      turnBodyRel(goDir);
      if (ballDist < 0.6)
        playMotion(forwardsMotion);
      else
        playMotion(forwards50Motion);
    }
  }
}
