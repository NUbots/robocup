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
//#include <boost/shared_ptr.hpp>

#include "NAOWebots.h"
#include "../../Motion/NUMotion.h"

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

    int playerID = (port % 10) + 1;
    Player *player = NULL;
    
    nubot = new NAOWebots(playerID);
    //nubot = boost::shared_ptr<NUbot>(new NAOWebots());
    cout << "main: nubot: " << nubot << endl;
    
#ifdef USE_VISION
    vision = new Vision();
#endif
#ifdef USE_LOCALISATION
    localisation = new Localisation();
#endif
#ifdef USE_BEHAVIOUR
    behaviour = new Behaviour();
#endif
#ifdef USE_MOTION
    motion = new NUMotion();
#endif
#ifdef USE_NETWORK
    network = new Network();
#endif
    
    string name;
    int number;
    nubot->getName(name);
    nubot->getNumber(number);
    cout << "main: Name: " << name << " Num: " << number << endl;
    
    nubot->test();

    // choose GoalKepper/FieldPlayer role according to last digit of URBI port number
    player = new FieldPlayer(playerID);

    player->run();
    delete player;
}
