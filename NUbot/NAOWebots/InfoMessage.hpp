#ifndef INFO_MESSAGE_HPP
#define INFO_MESSAGE_HPP

//-----------------------------------------------------------------------------
//  File:         InfoMessage class (to be used in a C++ Webots controllers)
//  Description:  Example of data packet that can be sent between robots
//                to support e.g. ball and teammates localization, etc.
//  Project:      Robotstadium, the online robot soccer competition
//  Author:       Yvan Bourquin - www.cyberbotics.com
//  Date:         May 4, 2008
//  Changes:      
//-----------------------------------------------------------------------------

// You should change the magic keyword in order to identify your own info messages
// this is necessary, in order to avoid spam messages ...
#define INFO_MESSAGE_STRUCT_HEADER "YourMagicKeywordHere"

struct InfoMessage {
  // header to identify the structure (not null-termination)
  char header[sizeof(INFO_MESSAGE_STRUCT_HEADER) - 1];
  int playerID;            // 0: goalkeeper, 1: field player 1, etc.
  double distanceToBall;   // my rough guess ...

// and anything else you want:
//   double myLocation[2];
//   double ballLocation[2];
//   bool fallen;
//   etc.
};

#endif
