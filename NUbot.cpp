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
    #ifdef TARGET_IS_NAOWEBOTS
        platform = new NAOWebots(argc, argv);
    #endif
    #ifdef TARGET_IS_NAO
        platform = new NAO();
    #endif
    #ifdef TARGET_IS_CYCLOID
        platform = new Cycloid();
    #endif
    
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
}

NUbot::~NUbot()
{
    
}

void NUbot::run()
{
    //while (true) {};
    return;
}

