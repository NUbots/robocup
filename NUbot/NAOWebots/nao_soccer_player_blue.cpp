//---------------------------------------------------------------------------------------
//  File:         nao_soccer_player_blue.cpp (to be used in a Webots controllers)
//  Description:  This is a full C++ controller example for Robotstadium
//                It selects and runs the correct player type: FieldPlayer or GoalKeeper
//  Project:      Robotstadium, the online robot soccer competition
//  Author:       Yvan Bourquin - www.cyberbotics.com
//  Date:         May 11, 2009
//  Changes:      
//---------------------------------------------------------------------------------------

#include "FieldPlayer.hpp"
#include "GoalKeeper.hpp"
#include <iostream>
#include <cstring>
#include <cstdlib>

using namespace std;

int main(int argc, const char *argv[]) {

  // find URBI port number (e.g. -p 54001) in controllerArgs
  int port = -1;
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
      port = atoi(argv[i + 1]);
      break;
    }
  }

  if (port == -1) {
    cout << "Error: could not find port number in controllerArgs" << endl;
    return 0;
  }

  int playerID = port % 10;
  Player *player = NULL;

  // choose GoalKepper/FieldPlayer role according to last digit of URBI port number
  if (playerID == 0)
    player = new GoalKeeper(playerID);
  else
    player = new FieldPlayer(playerID);

  player->run();
  delete player;
}
