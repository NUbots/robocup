/*! @file Sensors.h
    @brief Declaration of a base sensor class to store sensor data in a platform independent way

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

#ifndef SENSORS_H
#define SENSORS_H

#include <vector>
#include <pthread.h>
#include <iostream>

using namespace std;

/*! @brief Container for a single sensor
 */
struct sensor_type 
{
    vector<float> data;         //!< the sensor values
    vector<float> sd;           //!< standard deviation for each sensor value sensor
    bool isValid;               //!< true, if data is valid, false if not
    bool isCalculated;          //!< true, if data has been calculated, false if the data is direct from a sensor
    long double timestamp;      //!< the unix timestamp of the data
};

/*! @brief Base sensor storage class
 */
class Sensors
{
public:
    Sensors();
    ~Sensors();
    
private:
    
};

#endif