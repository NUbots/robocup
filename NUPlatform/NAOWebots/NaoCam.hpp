#ifndef NAO_CAM_HPP
#define NAO_CAM_HPP

//-----------------------------------------------------------------------------
//  File:         NaoCam C++ class (to be used in a Webots controllers)
//  Description:  Camera with top/bottom selection and simple blob localization capability
//  Project:      Robotstadium, the online robot soccer competition
//  Author:       Yvan Bourquin - www.cyberbotics.com
//  Date:         May 5, 2008
//  Changes:      
//-----------------------------------------------------------------------------

#include <webots/Camera.hpp>

namespace webots {
  class Robot;
  class Servo;
}

using namespace webots;
using namespace std;

class NaoCam : public Camera {
public:
  // value when object direction or elevation is unknown
  static const double UNKNOWN;

  // target goal colors
  enum { SKY_BLUE, YELLOW };

  // constructor
  NaoCam(const string &name, int goalColor, Robot *robot);

  // find ball and goal in most recent image
  void processImage();

  // direction and elevation angles are indicated in radians
  // with respect to the camera focal (or normal) line
  // therefore a direction angle will not exceed +/- half the fieldOfView
  // a positive direction is towards the right of the camera image
  // a positive elevation is towards the top of the camera image
  double getBallDirectionAngle() const { return ballDirectionAngle; }
  double getBallElevationAngle() const { return ballElevationAngle; }
  double getGoalDirectionAngle() const { return goalDirectionAngle; }

  // selection top or bottom camera (the top camera is selected initially)
  void selectTop();
  void selectBottom();
  double getOffsetAngle() const { return offsetAngle; }
  bool isTopSelected() const { return offsetAngle == 0.0; }

  // 40 degrees angle between top and bottom camera axes
  static const double OFFSET_ANGLE;

protected:
  void findColorBlob(const double ref[3], double tolerance, double &direction, double &elevation);

private:
  const unsigned char *image;
  double fov;  // field of view
  int width;   // pixel width
  int height;  // pixel height
  int goalColor;
  double ballDirectionAngle;
  double ballElevationAngle;
  double goalDirectionAngle;
  Servo *cameraSelect;
  double offsetAngle;
};

#endif
