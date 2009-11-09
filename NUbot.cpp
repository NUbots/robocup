/*! @file NUbot.cpp
    @brief Implementation of top-level NUbot class

    @author Jason Kulk
 
 Copyright (c) 2009 Jason Kulk
 
 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "NUbot.h"
#ifdef TARGET_IS_NAOWEBOTS
    #include "NUPlatform/NAOWebots/NAOWebots.h"
#endif
#ifdef TARGET_IS_NAO
    #include "NUPlatform/NAO/NAO.h"
#endif
#ifdef TARGET_IS_CYCLOID
    #include "NUPlatform/Cycloid/Cycloid.h"
#endif

NUbot::NUbot(int argc, const char *argv[])
{
    // Construct the right Platform
    #ifdef TARGET_IS_NAOWEBOTS
        platform = new NAOWebots(argc, argv);
    #endif
    #ifdef TARGET_IS_NAO
        platform = new NAO();
    #endif
    #ifdef TARGET_IS_CYCLOID
        platform = new Cycloid();
    #endif
    
    // Construct each enabled module 
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
    
    // NUbot threads:
    // You can't assume that these threads are syncronised, or running at any particular relative rate 
    // I need a thread which links BodySensors->NUMotion in a real-time thread.
    //      while (true):
    //          wait for new data
    //          data = nubot->platform->sensors->getData()                // I should not deep copy the data here
    //                cmds = nubot->motion->process(data)                       // it is up to motion to decide whether it should deep copy
    //          nubot->platform->actuators->process(cmds)
    //
    // I need a thread which links Image->Vision->Localisation->Behaviour->Motion in a non-real-time thread.
    //      while (true):
    //          wait for new image
    //          image = nubot->platform->camera->getData()
    //          data = nubot->platform->sensors->getData()                // I should not deep copy the data here
    //                 odometry = nubot->motion->getData()                       // There is no deep copy here either
    //      gamectrl, teaminfo = nubot->network->getData()
    //      
    //          fieldobj = nubot->vision->process(image, data, gamectrl)
    //          wm = nubot->localisation->process(fieldobj, teaminfo, odometry, gamectrl)
    //        actions = nubot->behaviour->process(wm, gamectrl)
    //                     nubot->motion->process(actions)                // I don't think anything needs to be returned here
    
    // so each thread is passed the nubot, and does it that way
}

NUbot::~NUbot()
{
    delete platform;
    #ifdef USE_VISION
        delete vision;
    #endif
    #ifdef USE_LOCALISATION
        delete localisation;
    #endif
    #ifdef USE_BEHAVIOUR
        delete behaviour;
    #endif
    #ifdef USE_MOTION
        delete motion;
    #endif
    #ifdef USE_NETWORK
        delete network;
    #endif
}

/*! @brief Brief description
    
 Detailed description
 */
void NUbot::run()
{
    BodySensors* data = new BodySensors();
    Action* action = new Action();
    ActuatorCommands* result;
    result = motion->process(data, action);
    //while (true) {};
    return;
}

