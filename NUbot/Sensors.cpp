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
}

/*! @breif Default destructor
 */
Sensors::~Sensors()
{
}

/*void Sensors::test()
{
    cout << "Sensors::test()" << endl;
    sensor_type blank;
    blank.timestamp = 0;
    vector<sensor_type> testsensors(10000, blank);
    testsensors[3].timestamp = 12345.456;
    testsensors[10].timestamp = 15606.12;
    
    cout << "testsensors[3].timestamp" << testsensors[3].timestamp << "testsensors[10].timestamp" << testsensors[10].timestamp << "whoknows" << testsensors[100].timestamp;
}*/

