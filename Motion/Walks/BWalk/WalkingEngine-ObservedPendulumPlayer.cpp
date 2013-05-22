/**
* @file WalkingEngine.cpp
* Implementation of a module that creates the walking motions
* @author Colin Graf
*/

#include <cstdio>
#include <assert.h>
#include <cmath>

#include "WalkingEngine.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Tools/Math/General.h"
#include "Kinematics/NUInverseKinematics.h"
#include "Requirements/MassCalibration.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Requirements/ForwardKinematic.h"
#include "NUPlatform/NUPlatform.h"
#include "debug.h"
#include "debugverbositynumotion.h"

//#include "Tools/InverseKinematic.h"
//#include "Tools/MessageQueue/InMessage.h"
//#include "Tools/Streams/InStreams.h"
//#include "Tools/Settings.h"
//#include "Tools/Math/Matrix.h"
//#include "Platform/SoundPlayer.h"

template<typename T> ostream& operator<<(ostream& output, const Vector3<T>& v)
{
    output << "[";
    output << v.x << ",";
    output << v.y << ",";
    output << v.z;
    output << "]";
    return output;
}

template<typename T> ostream& operator<<(ostream& output, const Vector2<T>& v)
{
    output << "[";
    output << v.x << ",";
    output << v.y;
    output << "]";
    return output;
}

void WalkingEngine::ObservedPendulumPlayer::init(StepType stepType, float t, SupportLeg supportLeg, const Vector2<>& r, const Vector2<>& x0, const Vector2<>& k, float deltaTime)
{


  Parameters& p = walkingEngine->p;

  active = true;
  launching = true;

  this->supportLeg = supportLeg;

  this->l = this->al = Vector3<>();
  this->s = StepSize();
  this->type = stepType;

  this->k = k;
  this->te = p.te;
  this->tb = -p.te;
  this->t = this->tb + t;
  this->xv0 = Vector2<>();
  this->x0 = x0;
  this->r = r;
  this->originalRX  = r.x;
  this->c = Vector2<>();


  this->sXLimit.max = p.speedMax.translation.x * (1.1f * 0.5f);
  this->sXLimit.min = p.speedMaxBackwards * (-1.1f * 0.5f);
  this->rXLimit.max = this->r.x + p.walkRefXSoftLimit.max;
  this->rXLimit.min = this->r.x + p.walkRefXSoftLimit.min;
  this->rYLimit.max = p.walkRefY + p.walkRefYLimit.max;
  this->rYLimit.min = p.walkRefY + p.walkRefYLimit.min;

  cov = b_human::Matrix4x4f(
          Vector4f(p.observerProcessDeviation[0]*p.observerProcessDeviation[0], 0.f, p.observerProcessDeviation[0] * p.observerProcessDeviation[2], 0.f),
          Vector4f(0.f, p.observerProcessDeviation[1]*p.observerProcessDeviation[1], 0.f, p.observerProcessDeviation[1] * p.observerProcessDeviation[3]),
          Vector4f(p.observerProcessDeviation[0] * p.observerProcessDeviation[2], 0.f, p.observerProcessDeviation[2]*p.observerProcessDeviation[2], 0.f),
          Vector4f(0.f, p.observerProcessDeviation[1] * p.observerProcessDeviation[3], 0.f, p.observerProcessDeviation[3]*p.observerProcessDeviation[3]));

  generateNextStepSize();

  computeSwapTimes(this->tb, 0.f, 0.f, 0.f);

  computeRefZmp(this->tb, r.x, 0.f, 0.f);

}

void WalkingEngine::ObservedPendulumPlayer::applyCorrection(const Vector3<>& leftError, const Vector3<>& rightError, float deltaTime)
{
  Parameters& p = walkingEngine->p;
  Vector3<> error;
  switch(p.observerErrorMode)
  {
  case Parameters::indirect:
    error = supportLeg != left ? leftError : rightError;
    break;
  case Parameters::direct:
    error = supportLeg == left ? leftError : rightError;
    break;
  case Parameters::mixed:
  {
    float x = (t - tb) / (te - tb);
    if(x < 0.5f)
      x = 0.f;
    else
      x = (x - 0.5f) * 2.f;
    if(supportLeg != left)
      x = 1.f - x;
    error = leftError * (1.f - x) + rightError * x;
  }
  default:
    error = (leftError + rightError) * 0.5f;
    break;
  }

  static const b_human::Matrix2x4f c(Vector2f(1, 0), Vector2f(0, 1), Vector2f(), Vector2f());
  static const b_human::Matrix4x2f cTransposed = c.transpose();
  static const b_human::Matrix4x4f a(Vector4f(1, 0, 0, 0), Vector4f(0, 1, 0, 0),
                            Vector4f(deltaTime, 0, 1, 0), Vector4f(0, deltaTime, 0, 1));
  static const b_human::Matrix4x4f aTransponsed = a.transpose();

  cov = a * cov * aTransponsed;

  for(int i = 0; i < 4; ++i)
    cov[i][i] += p.observerProcessDeviation[i]*p.observerProcessDeviation[i];

  b_human::Matrix2x2f covPlusSensorCov = c * cov * cTransposed;
  Vector2f observerMeasurementDeviation = p.observerMeasurementDeviation;
  if(walkingEngine->instable)
    observerMeasurementDeviation = p.observerMeasurementDeviationWhenInstable;
  else if(next.s.translation.x > 0.f)
  {
    observerMeasurementDeviation.x += (p.observerMeasurementDeviationAtFullSpeedX.x - p.observerMeasurementDeviation.x) * abs(next.s.translation.x) / (p.speedMax.translation.x * 0.5f);
    observerMeasurementDeviation.y += (p.observerMeasurementDeviationAtFullSpeedX.y - p.observerMeasurementDeviation.y) * abs(next.s.translation.x) / (p.speedMax.translation.x * 0.5f);
  }
  covPlusSensorCov[0][0] += observerMeasurementDeviation[0]*observerMeasurementDeviation[0];
  covPlusSensorCov[1][1] += observerMeasurementDeviation[1]*observerMeasurementDeviation[1];
  b_human::Matrix4x2f kalmanGain = cov * cTransposed * covPlusSensorCov.invert();
  Vector2f innovation(error.x, error.y);
  Vector4f correction = kalmanGain * innovation;
  cov -= kalmanGain * c * cov;

  // compute updated xt and xvt
  Vector2<> xt(
    r.x + this->c.x * t + x0.x * cosh(k.x * t) + xv0.x * sinh(k.x * t) / k.x + correction[0],
    r.y + this->c.y * t + x0.y * cosh(k.y * t) + xv0.y * sinh(k.y * t) / k.y + correction[1]);
  Vector2<> xvt(
    this->c.x + k.x * x0.x * sinh(k.x * t) + xv0.x * cosh(k.x * t) + correction[2],
    this->c.y + k.y * x0.y * sinh(k.y * t) + xv0.y * cosh(k.y * t) + correction[3]);

  computeSwapTimes(t, xt.y, xvt.y, error.y);
  computeRefZmp(t, xt.x, xvt.x, error.x);
}
