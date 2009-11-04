/*! @file NAOWebots.cpp
    @brief Implementation of NAOWebots : NUbot (Robot) class

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

#include "NAOWebots.h"
#include "../Sensors.h"
#include "../Actuators.h"

NAOWebots::NAOWebots()
{
    cout << "NAOWebots::NAOWebots" << endl;
    cout << "NAOWebots::NAOWebots. this: " << this << endl;
    sensors = new Sensors();
    actuators = new Actuators();
}

NAOWebots::~NAOWebots()
{
}

void NAOWebots::getName(string& name)
{
    cout << "NAOWebots::getName" << endl;
    name = string("Susannah");
}

void NAOWebots::getNumber(int& number)
{
    cout << "NAOWebots::getNumber" << endl;
    number = 0;
}

void NAOWebots::test()
{
    cout << "NAOWebots::test() motion: " << motion << endl;
}

