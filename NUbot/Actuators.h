/*! @file Actuators.h
    @brief Declaration of a base actuator class for robot manipulation

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

#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <vector>
using namespace std;

/*! @brief Container for a single actuator
 
 This structure holds the desired actuator targets. The targets for position, speed, and stiffness 
 are specified for each time in times.
 
 The target positions, speeds, and stiffnesses may be interpolated between each of the specified times.
 */
struct actuator_type 
{
    string name;                        //!< the actuator's name
    int actuatorID;                     //!< the actuator's id
    vector<float> times;                //!< times for each target point
    vector<float> targetPositions;      //!< positions to reach at each time 
    vector<float> targetSpeeds;         //!< speed to travel at each time
    vector<float> targetStiffnesses;    //!< stiffness for each time
};

class Actuators
{
public:
    Actuators();
    ~Actuators();
};

#endif

