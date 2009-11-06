#include "NaoCam.hpp"
#include <webots/Robot.hpp>
#include <webots/Servo.hpp>
#include <cmath>

using namespace std;
using namespace webots;

const double NaoCam::UNKNOWN = -999.9;
const double NaoCam::OFFSET_ANGLE = 0.6981;

NaoCam::NaoCam(const string &name, int goalColor, Robot *robot) : Camera(name) {
  image = NULL;
  this->goalColor = goalColor;

  // keep at hand
  fov = getFov();
  width = getWidth();
  height = getHeight();

  // initially unknown
  ballDirectionAngle = UNKNOWN;
  ballElevationAngle = UNKNOWN;
  goalDirectionAngle = UNKNOWN;

  // get camera switch
  cameraSelect = robot->getServo("CameraSelect");
  offsetAngle = 0.0;
}

// normalize rgb components to be less dependant of lighting
static inline void rgbnorm(const unsigned char in[3], double out[3]) {
  double max = in[0];
  if (in[1] > max) max = in[1];
  if (in[2] > max) max = in[2];
  out[0] = in[0] / max;
  out[1] = in[1] / max;
  out[2] = in[2] / max;
}

// find a blob whose rgb components match [R G B] with given tolerance
// return blob center as angles in radian with respect to camera axis
void NaoCam::findColorBlob(const double ref[3], double tolerance, double &direction, double &elevation) {

  int length = width * height;  // number of pixels in the image
  int x = 0, y = 0;
  int npixels = 0;

  const unsigned char *p = image;
  for (int i = 0; i < length; i++, p+=3) {
    double pixel[3];
    rgbnorm(p, pixel);

    if (fabs(pixel[0] - ref[0]) < tolerance && fabs(pixel[1] - ref[1]) < tolerance && fabs(pixel[2] - ref[2]) < tolerance) {
      x += i % width;
      y += i / width;
      npixels++;
    }
  }

  if (npixels > 0) {
    direction = ((double)x / npixels / width - 0.5) * fov;
    elevation = -((double)y / npixels / height - 0.5) * fov;
  }
  else {
    direction = UNKNOWN;
    elevation = UNKNOWN;
  }
}

// get latest image and find blobs
void NaoCam::processImage() {

  // reference colors
  const double BALL_COLOR[3] = { 1.0, 0.5, 0.2 };
  const double BLUE_GOAL_COLOR[3] = { 0.1, 1.0, 1.0 };
  const double YELLOW_GOAL_COLOR[3] = { 1.0, 1.0, 0.1 };

  // latest image
  image = getImage();

  // find orange ball
  findColorBlob(BALL_COLOR, 0.3, ballDirectionAngle, ballElevationAngle);

  // find goal
  double dummy;
  switch (goalColor) {
  case SKY_BLUE:
    findColorBlob(BLUE_GOAL_COLOR, 0.3, goalDirectionAngle, dummy);
    break;
  case YELLOW:
    findColorBlob(YELLOW_GOAL_COLOR, 0.3, goalDirectionAngle, dummy);
    break;
  }

  //cout << "camera: ball: dir: " << ballDirectionAngle << " elev: " << ballElevationAngle << endl;
  //cout << "camera: goal: dir: " << goalDirectionAngle << endl;
}

void NaoCam::selectTop() {
  cameraSelect->setPosition(0.0);
  offsetAngle = 0.0;
}

void NaoCam::selectBottom() {
  cameraSelect->setPosition(OFFSET_ANGLE);
  offsetAngle = OFFSET_ANGLE;
}

