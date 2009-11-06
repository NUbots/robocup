#ifndef GOAL_KEEPER_HPP
#define GOAL_KEEPER_HPP

//-----------------------------------------------------------------------------
//  File:         GoalKeeper class (to be used in a Webots controllers)
//  Description:  Goal keeper (not a field player !)
//  Project:      Robotstadium, the online robot soccer competition
//  Author:       Yvan Bourquin - www.cyberbotics.com
//  Date:         May 8, 2009
//  Changes:      
//-----------------------------------------------------------------------------

#include "Player.hpp"

class GoalKeeper : public Player {
public:
  GoalKeeper(int playerID);
  virtual ~GoalKeeper();

  // overridden function
  virtual void run();

protected:

private:
  Motion *sideStepLeftMotion, *sideStepRightMotion, *forwards50Motion, *backwardsMotion;
  int rightStepsCount;

  void stepRight();
  void stepLeft();
};

#endif
