/*! 
    @file TestConfig.cpp
    @brief This is the cpp file for the testing of the configuration system for the NUbots.
  
    @author Sophie Calland, Mitchell Metcalfe
 
  Copyright (c) 2012 Sophie Calland, Mitchell Metcalfe
 
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

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <exception>

#include "ConfigManager.h"
//#include "ConfigStorageManager.h"

using namespace ConfigSystem;


int main(void)
{
    // // Create path array for config system.
    // // Note: this should *definitely* be internal to the config system
    // // (perhaps only specify a base path + let the constructor handle 
    // // the filenames itself?).
    // std::string fNames[] = 
    //     {
    //         "ConfigurationFiles/behaviour.json"    ,
    //         "ConfigurationFiles/communication.json",
    //         "ConfigurationFiles/configuration.json",
    //         "ConfigurationFiles/localisation.json" ,
    //         "ConfigurationFiles/locomotion.json"   ,
    //         "ConfigurationFiles/vision.json"       ,
    //     };
    // //Read all config files. 
    ConfigManager config("defaultConfig");

    double sharpness;
    config.readDoubleValue("vision.Camera", "Sharpness", sharpness);
    std::cout << "Read: vision.Camera.Sharpness = " << sharpness << std::endl;
    
    sharpness += 5;
    
    bool stored = config.storeDoubleValue("vision.Camera", "Sharpness", sharpness);

    std::cout << "Successfully stored: " << stored << std::endl;

    std::cout << "Exit program." << std::endl;

    config.saveConfiguration("newConfig");

    // // A property tree that holds ConfigParameter objects
    // boost::property_tree::basic_ptree<std::string, ConfigParameter, std::less<std::string> >
    //  cpTree;
    // ConfigParameter newCP(vt_double);
    // cpTree.put("root", newCP);

    return 0;
}
