/*! @file NUbot.h
    @brief Declaration of a base nubot class.

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

#include <string>
//#include <boost/shared_ptr.hpp>

using namespace std;

#include "../config.h"

#include "NUCamera.h"
#include "Sensors.h"
#include "Actuators.h"

class NUbot
{
// Functions:
public:
    NUbot();
    virtual ~NUbot();
    
    virtual void getName(string& name);
    virtual void getNumber(int& number);
    
    virtual void test();
protected:
private:
    
// Members:
public:
    NUCamera* camera;
    Sensors* sensors;
    Actuators* actuators;
protected:
    string m_name;
    int m_number;
private:
    
};

extern NUbot* nubot;
//static NUbot* nubot = NULL;
//static boost::shared_ptr<NUbot> nubot;

#endif