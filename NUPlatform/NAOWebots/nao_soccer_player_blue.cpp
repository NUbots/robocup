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

#include "NUbot.h"
#include "NAOWebots.h"
#include "Motion/NUMotion.h"

using namespace std;

int main(int argc, const char *argv[]) 
{
    NUbot* nubot = new NUbot(argc, argv);
    nubot->run();
    delete nubot;
}
