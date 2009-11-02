#include "GoalKeeper.hpp"
#include "NaoCam.hpp"
#include <webots/Servo.hpp>
#include <webots/utils/Motion.hpp>

using namespace webots;

GoalKeeper::GoalKeeper(int playerID) : Player(playerID) {
  rightStepsCount = 0;

  // load motions
  forwards50Motion = new Motion("../motions/Forwards50.motion");
  backwardsMotion = new Motion("../motions/Backwards.motion");
  sideStepLeftMotion = new Motion("../motions/SideStepLeft.motion");
  sideStepRightMotion = new Motion("../motions/SideStepRight.motion");

  // move arms
  Servo *leftShoulderRoll = getServo("LShoulderRoll");
  Servo *rightShoulderRoll = getServo("RShoulderRoll");
  leftShoulderRoll->setPosition(1.5);
  rightShoulderRoll->setPosition(-1.5);
}

GoalKeeper::~GoalKeeper() {
  delete forwards50Motion;
  delete backwardsMotion;
  delete sideStepLeftMotion;
  delete sideStepRightMotion;
}

void GoalKeeper::stepRight() {
  playMotion(sideStepRightMotion);
  rightStepsCount++;
}

void GoalKeeper::stepLeft() {
  playMotion(sideStepLeftMotion);
  rightStepsCount--;
}

void GoalKeeper::run() {
  step(SIMULATION_STEP);
  step(SIMULATION_STEP);

  while (true) {
    getUpIfNecessary();

    // loop until ball becomes visible
    while (getBallDirection() == NaoCam::UNKNOWN) {
      getUpIfNecessary();
      if (getBallDirection() != NaoCam::UNKNOWN) break;
      headScan();
    }

    double ballDir = getBallDirection();
    double ballDist = getBallDistance();

    if (ballDist < 0.8 && ballDir > -0.15 && ballDir < 0.15) {
      // ball is close and in front: try to kick it
      playMotion(forwards50Motion);

      // move backwards to goal
      for (int i = 0; i < 5; i++)
        playMotion(backwardsMotion);

    }
    else if (ballDist < 2.0) {
      // if the ball is within 2 meters
      // step right/left in the direction of the ball
      if (ballDir > 0.1 && rightStepsCount < 8)
        stepRight();
      else if (ballDir < -0.1 && rightStepsCount > -8)
        stepLeft();
    }
    else {
      // the ball is quite far: step back to the center of the goal
      if (rightStepsCount < 0)
        stepRight();
      else if (rightStepsCount > 0)
        stepLeft();
    }

    runStep();
  }
}
