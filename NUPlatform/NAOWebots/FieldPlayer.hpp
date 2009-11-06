#ifndef FIELD_PLAYER_HPP
#define FIELD_PLAYER_HPP

//-----------------------------------------------------------------------------
//  File:         FieldPlayer C++ class (to be used in a Webots controllers)
//  Description:  Field player (not a goalkeeper !)
//  Project:      Robotstadium, the online robot soccer competition
//  Author:       Yvan Bourquin - www.cyberbotics.com
//  Date:         May 8, 2009
//  Changes:      
//-----------------------------------------------------------------------------

#include "Player.hpp"

class FieldPlayer : public Player {
public:
  FieldPlayer(int playerID);
  virtual ~FieldPlayer();

  // overridden function
  virtual void run();

protected:
  virtual void runStep();

private:
  // motion files
  Motion *backwardsMotion, *forwardsMotion, *forwards50Motion, *turnRight40Motion, *turnLeft40Motion;
  Motion *turnRight60Motion, *turnLeft60Motion, *turnLeft180Motion, *sideStepRightMotion, *sideStepLeftMotion;

  // guessed goal direction (with respect to front direction of robot body)
  double goalDir;

  void turnBodyRel(double angle);
  void turnRight60();
  void turnLeft60();
  void turnRight40();
  void turnLeft40();
  void turnLeft180();
};

#endif
