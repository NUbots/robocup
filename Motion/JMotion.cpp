/*! @file JMotion.cpp
    @brief Implementation of motion class

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

#include <iostream>
using namespace std;

#include "../NUbot/NUbot.h"
#include "JMotion.h"

JMotion* motion = NULL;


JMotion::JMotion()
{
    cout << "JMotion::JMotion" << endl;
    string name;
    if (nubot != NULL)      // Note. the pointer nubot will be NULL until the NUbot constructor has RETURNED!
    {
        nubot->getName(name);
        cout << "JMotion::JMotion nubot->getName: " << name << endl;
    }
    else 
    {
        cout << "JMotion::JMotion nubot" << nubot << endl;
    }

}

