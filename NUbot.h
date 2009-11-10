/*! @file NUbot.h
    @brief Declaration of top-level NUbot class.

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

#ifndef NUBOT_H
#define NUBOT_H

#include "config.h"
#include "NUPlatform/NUPlatform.h"

// Selectively include modules depending on config.h
#ifdef USE_VISION
    #include "Vision/Vision.h"
#endif

#ifdef USE_LOCALISATION
    #include "Localisation/Localisation.h"
#endif

#ifdef USE_BEHAVIOUR
    #include "Behaviour/Behaviour.h"
#endif

#ifdef USE_MOTION
    #include "Motion/NUMotion.h"
#endif

#ifdef USE_NETWORK
    #include "Network/Network.h"
#endif

#include <pthread.h>

/*! @brief The top-level file
 */
class NUbot
{
// Functions:
public:
    NUbot(int argc, const char *argv[]);
    ~NUbot();
    void run();
    
    int signalMotion();
    int waitForNewMotionData();
    
    int signalVision();
    int waitForNewVisionData();
private:
    void createThreads();
    
public:
    
private:
    NUPlatform* platform;               //!< interface to robot platform
    #ifdef USE_VISION
        Vision* vision;                 //!< vision module
    #endif
    #ifdef USE_LOCALISATION
        Localisation* localisation;     //!< localisation module
    #endif
    #ifdef USE_BEHAVIOUR
        Behaviour* behaviour;           //!< behaviour module
    #endif
    #ifdef USE_MOTION
        NUMotion* motion;               //!< motion module
    #endif
    #ifdef USE_NETWORK
        Network* network;               //!< network module
    #endif
    
    pthread_mutex_t mutexMotionData;
    pthread_cond_t condMotionData;      
    pthread_t threadMotion;             //!< thread containing the direct sensory links to motion (cerebellum)
    pthread_mutex_t mutexVisionData;
    pthread_cond_t condVisionData;    
    pthread_t threadVision;             //!< thread containing vision and higher-level though processes (cerebrum)
};

#endif