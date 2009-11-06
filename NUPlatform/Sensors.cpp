/*! @file Sensors.cpp
    @brief Partial implementation of base sensor class

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

#include "Sensors.h"
#include <iostream>
using namespace std;

/*! @brief Default constructor
 */
Sensors::Sensors()
{
    cout << "Sensors::Sensors" << endl;
    devices.reserve(2);
    sensor_type* blank = new sensor_type;
    blank->timestamp = 13;
    devices.push_back(blank);
    Sensor1 = devices[0];
    blank = new sensor_type;
    blank->timestamp = 22;
    devices.push_back(blank);           // this uses a copy constructor to append to the end of the vector.
    Sensor2 = devices[1];               // this also invokes a copy constructor, so Sensor2 and devices[1] are not the same object!!!
    
    
    // Option 1. vector<sensor_type> devices
    // Pros:    no pointers
    //          nubot->sensors->devices[0].timestamp
    // Option 2. vector<sensor_type*> devices
    // Pros:    nubot->sensors->devices[0]->timestamp AND nubot->sensors->JointPositions->timestamp;
    
    cout << "Sensors::Sensors: ";
    for (int i=0; i<devices.size(); i++)
    {
        cout << devices[i]->timestamp << ", ";
    }
    cout << endl;
    
    devices[0]->timestamp = 666;
    Sensor2->timestamp = 555;
    
    cout << "Sensors::Sensors: ";
    for (int i=0; i<devices.size(); i++)
    {
        cout << devices[i]->timestamp << ", ";
    }
    cout << endl;
    
}

/*! @brief Default destructor
 */
Sensors::~Sensors()
{
}

/*! @brief Updates the sensor data
 */
void Sensors::update()
{
    
}

