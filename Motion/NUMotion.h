/*! @file NUMotion.h
    @brief Declaration of motion class
 
    @class NUMotion
    @brief A module to provide motion
 
    This module should be able to do 3 things
        - Walk (Speed, Position)
        - Head (Look at Point, Nod, Pan, etc)
        - Special (Kicks, Saves, Getups etc)
 
    So my arguement for motion, is that; every motion involves a movement to a point, and then an execution of a 'motion action' at that point.
    The motion action itself would have a target and a speed.
    So
    actionName                                      <= action name
    vector<float> speed, vector<position>           <= relative position where the action is to be performed
    vector<actionspeed>, vector<actionposition>     <= action target
 
    So, a MotionAction needs to have
        - A Type (Walk, Head, Special)
        - Data
            - WALK: vector<speed>, vector<position>: x, y, theta
            - HEAD: vector<speed>, vector<position>: angleYaw, anglePitch
            - SPECIAL: 
                - Kick: (distance, bearing) to ball, (distance, bearing) to target (if distance, bearing to ball not in range walk to closest point instead)
                - Save: (distance, bearing) to ball, ?
                - Getup: None

    If I use inheritance:
    Action
        -- MotionAction
            -- WalkAction
            -- HeadAction
            -- SpecialAction
        -- LightAction
        -- CameraAction
        -- SoundAction
    Then I either need to make all functions required by each virtual in Action, or use casting. Casting should be avoided.
    Alternativly just have a single action class that encapsulates everything; Well this is almost.
 
    if (ActionType == Action::Motion)
        position = action.getMovePosition()         // get the position where the action is to be performed
        speed = action.getMoveSpeed()               // get the speed
 
 
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

#ifndef NUMOTION_H
#define NUMOTION_H

#include "Behaviour/Action.h"

class ActuatorCommands
{
public:
    ActuatorCommands() {};
    ~ActuatorCommands() {};
};

class BodyData
{
public:
    BodyData() {};
    ~BodyData() {};
};

class NUMotion
{
public:
    NUMotion();
    ~NUMotion();
    
    ActuatorCommands* process(BodyData* data);
    void process(Action* actions);
protected:
private:
public:
protected:
private:
};

#endif