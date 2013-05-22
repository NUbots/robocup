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


inline float saveAsinh(float xf)
{
  double x = xf; // yes, we need double here
#ifdef _MSC_VER
  return float(log(x + sqrt(x * x + 1.)));
#else
  return float(asinh(x));
#endif
}

inline float saveAcosh(float xf)
{
  //assert(xf >= 1.f);
  double x = xf; // yes, we need double here
  if(x < 1.)
    return 0.000001f;
#ifdef WIN32
  x = log(x + sqrt(x * x - 1.));
#else
  x = acosh(x);
#endif
  if(x < 0.000001)
    return 0.000001f;
  return float(x);
}

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

void WalkingEngine::PendulumPlayer::seek(float deltaT)
{
  t += deltaT;
  if(t >= 0.f)
  {
    launching = false;
    if(t >= te)
    {
      if(type == toStand || type == toStandLeft || type == toStandRight)
      {
        active = false;
        return;
      }

      float const xTeY = r.y + c.y * te + x0.y * cosh(k.y * te) + xv0.y * sinh(k.y * te) / k.y;
      float const xvTeY = c.y + k.y * x0.y * sinh(k.y * te) + xv0.y * cosh(k.y * te);
      float const xTeX = r.x + c.x * te + x0.x * cosh(k.x * te) + xv0.x * sinh(k.x * te) / k.x;
      float const xvTeX = c.x + k.x * x0.x * sinh(k.x * te) + xv0.x * cosh(k.x * te);

      supportLeg = supportLeg == left ? right : left;
      t = next.tb + (t - te);
      (PendulumParameters&)*this = next;
      generateNextStepSize();

      computeSwapTimes(tb, xTeY - s.translation.y, xvTeY, 0.f);
      computeRefZmp(tb, xTeX - s.translation.x, xvTeX, 0.f);
    }
  }
}

void WalkingEngine::PendulumPlayer::generateNextStepSize()
{
  walkingEngine->generateNextStepSize(supportLeg == right ? left : right , type, kickType, next);
}

void WalkingEngine::PendulumPlayer::computeSwapTimes(float t, float xt, float xvt, float errory)
{
  if(te - t < 0.005f)
    return;

  switch(type)
  {
  case toStand:
  case toStandLeft:
  case toStandRight:
  case fromStand:
  case fromStandLeft:
  case fromStandRight:
  {
    float const xte = next.s.translation.y + next.xtb.y;
    float const xvte = next.xvtb.y;

    // r * 1 + c * te + x0 * cosh(k * te)     + xv0 * sinh(k * te) / k  = xte
    //       + c * 1  + x0 * k * sinh(k * te) + xv0 * cosh(k * te)      = xvte
    // r * 1 + c * t  + x0 * cosh(k * t)      + xv0 * sinh(k * t) / k   = xt
    //       + c * 1  + x0 * k * sinh(k * t)  + xv0 * cosh(k * t)       = xvt

    b_human::Matrix<4, 4> a(
      Vector<4>(1.f, 0.f, 1.f, 0.f),
      Vector<4>(te, 1.f, t, 1.f),
      Vector<4>(cosh(k.y * te), k.y * sinh(k.y * te), cosh(k.y * t), k.y * sinh(k.y * t)),
      Vector<4>(sinh(k.y * te) / k.y, cosh(k.y * te), sinh(k.y * t) / k.y, cosh(k.y * t)));
    Vector<4> b(xte, xvte, xt, xvt);

    Vector<4> x;
    if(!a.solve(b, x))
    {
      assert(false);
      return;
    }

    r.y = x[0];
    c.y = x[1];
    x0.y = x[2];
    xv0.y = x[3];
  }
  return;
  default:
    break;
  }

  Parameters& p = walkingEngine->p;

  if(errory != 0.f && walkingEngine->balanceStepSize.y != 0.f && !walkingEngine->m_pedantic && kickType == KickPlayer::none /*&& !walkingEngine->theMotionRequest.walkRequest.pedantic*/)
  {
    assert(next.xv0.y == 0.f);
    float sy = next.xtb.y * -2.f;
    sy += errory * (walkingEngine->instable ? p.balanceStepSizeWhenInstable.y : walkingEngine->balanceStepSize.y);
    next.tb = -saveAcosh((sy * -0.5f - next.r.y) / next.x0.y) / next.k.y;
    next.xtb.y =  sy * -0.5f;
    next.xvtb.y =  next.x0.y * next.k.y * sinh(next.k.y * next.tb);
  }

  assert(next.xv0.y == 0.f);
  //float const xte = next.s.translation.y + next.xtb.y;
  float const xvte = next.xvtb.y;

  //           x0.y * k * sinh(k * te) + xv0.y * cosh(k * te)     = xvte
  // r.y * 1 + x0.y * cosh(k * t)      + xv0.y * sinh(k * t) / k  = xt
  //           x0.y * k * sinh(k * t)  + xv0.y * cosh(k * t)      = xvt

  b_human::Matrix<3, 3> a(
    Vector<3>(0.f, 1.f, 0.f),
    Vector<3>(k.y * sinh(k.y * te), cosh(k.y * t), k.y * sinh(k.y * t)),
    Vector<3>(cosh(k.y * te), sinh(k.y * t) / k.y, cosh(k.y * t)));
  Vector<3> b(xvte, xt, xvt);

  Vector<3> x;
  if(!a.solve(b, x))
  {
    assert(false);
    return;
  }

  r.y = x[0];
  c.y = 0.f;
  x0.y = x[1];
  xv0.y = x[2];

  float newXte = r.y  + x0.y * cosh(k.y * te) + xv0.y * sinh(k.y * te) / k.y;
  next.s.translation.y = newXte - next.xtb.y;
}

void WalkingEngine::PendulumPlayer::computeRefZmp(float t, float xt, float xvt, float errorx)
{
  if(te - t < 0.005f)
    return;

  switch(type)
  {
  case toStand:
  case toStandLeft:
  case toStandRight:
  {
    float const xte = next.s.translation.x + next.xtb.x;
    float const xvte = next.xvtb.x;

    // r * 1 + c * te + x0 * cosh(k * te)     + xv0 * sinh(k * te) / k  = xte
    //       + c * 1  + x0 * k * sinh(k * te) + xv0 * cosh(k * te)      = xvte
    // r * 1 + c * t  + x0 * cosh(k * t)      + xv0 * sinh(k * t) / k   = xt
    //       + c * 1  + x0 * k * sinh(k * t)  + xv0 * cosh(k * t)       = xvt

    b_human::Matrix<4, 4> a(
      Vector<4>(1.f, 0.f, 1.f, 0.f),
      Vector<4>(te, 1.f, t, 1.f),
      Vector<4>(cosh(k.x * te), k.x * sinh(k.x * te), cosh(k.x * t), k.x * sinh(k.x * t)),
      Vector<4>(sinh(k.x * te) / k.x, cosh(k.x * te), sinh(k.x * t) / k.x, cosh(k.x * t)));
    Vector<4> b(xte, xvte, xt, xvt);

    Vector<4> x;
    if(!a.solve(b, x))
    {
      assert(false);
      return;
    }

    r.x = x[0];
    c.x = x[1];
    x0.x = x[2];
    xv0.x = x[3];
  }
  return;
  default:
    break;
  }

  Parameters& p = walkingEngine->p;
  if(errorx != 0.f && walkingEngine->balanceStepSize.x != 0.f && !walkingEngine->m_pedantic && kickType == KickPlayer::none  /*&& !walkingEngine->theMotionRequest.walkRequest.pedantic */)
  {
    assert(next.x0.x == 0.f);
    float sx = next.xv0.x * sinh(next.k.x * next.tb) / (-0.5f * next.k.x);
    sx += errorx * (walkingEngine->instable ? p.balanceStepSizeWhenInstable.x : walkingEngine->balanceStepSize.x);
    next.xv0.x = sx * -0.5f * next.k.x / sinh(next.k.x * next.tb);
    next.xtb.x = next.r.x + next.xv0.x * sinh(next.k.x * next.tb) / next.k.x;
    next.xvtb.x = next.xv0.x * cosh(next.k.x * next.tb);
  }


  assert(next.x0.x == 0.f);
  //float const xte = next.s.translation.x + next.xtb.x;
  float const xvte = next.xvtb.x;

  //           x0.x * k * sinh(k * te) + xv0.x * cosh(k * te)     = xvte
  // r.x * 1 + x0.x * cosh(k * t)      + xv0.x * sinh(k * t) / k  = xt
  //           x0.x * k * sinh(k * t)  + xv0.x * cosh(k * t)      = xvt

  b_human::Matrix<3, 3> a(
    Vector<3>(0.f, 1.f, 0.f),
    Vector<3>(k.x * sinh(k.x * te), cosh(k.x * t), k.x * sinh(k.x * t)),
    Vector<3>(cosh(k.x * te), sinh(k.x * t) / k.x, cosh(k.x * t)));
  Vector<3> b(xvte, xt, xvt);

  Vector<3> x;
  if(!a.solve(b, x))
  {
    assert(false);
    return;
  }

  r.x = x[0];
  c.x = 0.f;
  x0.x = x[1];
  xv0.x = x[2];

  float newXte = r.x +  x0.x * cosh(k.x * te) + xv0.x * sinh(k.x * te) / k.x;
  float newXvte = x0.x * k.x * sinh(k.x * te) + xv0.x * cosh(k.x * te);
  float newNextXvtb = newXvte;
  float newNextXv0 = newNextXvtb / cosh(next.k.x * next.tb);
  float newNextXtb = next.r.x + newNextXv0 * sinh(next.k.x * next.tb) / next.k.x;
  next.s.translation.x = newXte - newNextXtb;

  if(!rXLimit.isInside(r.x))
  {
    r.x = rXLimit.limit(r.x);

    // x0.x * cosh(k * t)      + xv0.x * sinh(k * t) / k  = xt - r.x
    // x0.x * k * sinh(k * t)  + xv0.x * cosh(k * t)      = xvt

    b_human::Matrix<2, 2> a(
      Vector<2>(cosh(k.x * t), k.x * sinh(k.x * t)),
      Vector<2>(sinh(k.x * t) / k.x, cosh(k.x * t)));
    Vector<2> b(xt - r.x, xvt);

    Vector<2> x;
    if(!a.solve(b, x))
    {
      assert(false);
      return;
    }

    x0.x = x[0];
    xv0.x = x[1];

    float newXte = r.x +  x0.x * cosh(k.x * te) + xv0.x * sinh(k.x * te) / k.x;
    float newXvte = x0.x * k.x * sinh(k.x * te) + xv0.x * cosh(k.x * te);
    float newNextXvtb = newXvte;
    float newNextXv0 = newNextXvtb / cosh(next.k.x * next.tb);
    float newNextXtb = next.r.x + newNextXv0 * sinh(next.k.x * next.tb) / next.k.x;
    //if(kickType == KickPlayer::none)
    next.s.translation.x = newXte - newNextXtb;
    if(type == unknown)
    {
      next.xv0.x = newNextXv0;
      next.xtb.x = newNextXtb;
      next.xvtb.x = newNextXvtb;
    }
  }

  if(!sXLimit.isInside(next.s.translation.x) && kickType == KickPlayer::none)
  {
    next.s.translation.x = sXLimit.limit(next.s.translation.x);

    // r + x0 * cosh(k * t)      + xv0 * sinh(k * t) / k                                                                     = xt
    //     x0 * k * sinh(k * t)  + xv0 * cosh(k * t)                                                                         = xvt
    //     x0 * k * sinh(k * te) + xv0 * cosh(k * te)          - nx0 * nk * sinh(nk * ntb) - nxv0 * cosh(nk * ntb)           = 0      // <=> xvte = nxvtb
    // r + x0 * cosh(k * te)     + xv0 * sinh(k * te) / k - nr - nx0 * cosh(nk * ntb)      - nxv0 * sinh(nk * ntb) / nk - ns = 0      // <=> xte - nxtb = ns

    // nx0 = 0

    // r + x0 * cosh(k * t)      + xv0 * sinh(k * t) / k                                = xt
    //     x0 * k * sinh(k * t)  + xv0 * cosh(k * t)                                    = xvt
    //     x0 * k * sinh(k * te) + xv0 * cosh(k * te)      - nxv0 * cosh(nk * ntb)      = 0
    // r + x0 * cosh(k * te)     + xv0 * sinh(k * te) / k  - nxv0 * sinh(nk * ntb) / nk = ns + nr

    b_human::Matrix<4, 4> a(
      Vector<4>(1.f, 0.f, 0.f, 1.f),
      Vector<4>(cosh(k.x * t), k.x * sinh(k.x * t), k.x * sinh(k.x * te), cosh(k.x * te)),
      Vector<4>(sinh(k.x * t) / k.x, cosh(k.x * t), cosh(k.x * te), sinh(k.x * te) / k.x),
      Vector<4>(0.f, 0.f, -cosh(next.k.x * next.tb), -sinh(next.k.x * next.tb) / next.k.x));
    Vector<4> b(xt, xvt, 0, next.s.translation.x + next.r.x);

    Vector<4> x;
    if(!a.solve(b, x))
    {
      assert(false);
      return;
    }

    r.x = x[0];
    x0.x = x[1];
    xv0.x = x[2];

    if(type == unknown)
    {
      next.xvtb.x = x0.x * k.x * sinh(k.x * te) + xv0.x * cosh(k.x * te);
      next.xv0.x = next.xvtb.x / cosh(next.k.x * next.tb);
      next.xtb.x = next.r.x + next.xv0.x * sinh(next.k.x * next.tb) / next.k.x;
    }
  }

  if(type == unknown)
  {
    Parameters& p = walkingEngine->p;
    rXLimit.max = originalRX + p.walkRefXLimit.max;
    if(rXLimit.max > p.walkRefXAtFullSpeedX + p.walkRefXLimit.max)
      rXLimit.max = p.walkRefXAtFullSpeedX + p.walkRefXLimit.max;
    rXLimit.min = originalRX + p.walkRefXLimit.min;
    type = normal;
  }
}

void WalkingEngine::PendulumPlayer::getStance(LegStance& stance, float* leftArmAngle, float* rightArmAngle, StepSize* stepOffset) const
{
  const float phase = (t - tb) / (te - tb);

  Parameters& p = walkingEngine->p;
  const float swingMoveFadeIn = phase < p.walkMovePhase.start ? 0.f :
                                phase > p.walkMovePhase.start + p.walkMovePhase.duration ? 1.f :
                                smoothShape((phase - p.walkMovePhase.start) / p.walkMovePhase.duration);
  const float swingMoveFadeOut = 1.f - swingMoveFadeIn;
  const float sl = phase < p.walkLiftPhase.start || phase > p.walkLiftPhase.start + p.walkLiftPhase.duration ? 0.f :
                   smoothShape((phase - p.walkLiftPhase.start) / p.walkLiftPhase.duration * 2.f);


  Vector3<> r(this->r.x + this->c.x * t, this->r.y + this->c.y * t, 0.f);
  Vector3<> refToCom(
    p.standComPosition.x + x0.x * cosh(k.x * t) + xv0.x * sinh(k.x * t) / k.x,
    x0.y * cosh(k.y * t) + xv0.y * sinh(k.y * t) / k.y,
    p.standComPosition.z);

  switch(type)
  {
  case toStandLeft:
  case toStandRight:
  case fromStandLeft:
  case fromStandRight:
  {
    const float ratio = smoothShape(type == toStandLeft || type == toStandRight ? 1.f - t / tb : 1.f - t / te);
    refToCom.z += ratio * (p.kickComPosition.z - p.standComPosition.z);
    refToCom.x += ratio * (p.kickComPosition.x - p.standComPosition.x);
//    std::cout << "case fromStandRight: refToCom = " << refToCom << std::endl;

  }
  break;
  default:
    refToCom += next.al * sl;
//    std::cout << "case default: refToCom = " << refToCom << std::endl;

    break;
  }

  if(supportLeg == left)
  {
    const Vector3<> rightStepOffsetTranslation = next.l * sl - (next.s.translation + s.translation) * swingMoveFadeOut;

    Vector3<> rightStepOffsetRotation = next.lRotation * sl;
    rightStepOffsetRotation.z += next.s.rotation * swingMoveFadeIn;
    const Vector3<> leftStepOffsetRotation(0.f, 0.f, s.rotation * swingMoveFadeOut);

    stance.leftOriginToCom = refToCom + r;
    stance.leftOriginToFoot = Pose3D(RotationMatrix(leftStepOffsetRotation), Vector3<>(0.f, p.standComPosition.y, 0.f));

    stance.rightOriginToCom = refToCom + r - next.s.translation;
    stance.rightOriginToFoot = Pose3D(RotationMatrix(rightStepOffsetRotation), Vector3<>(0.f, -p.standComPosition.y, 0.f) + rightStepOffsetTranslation);

    if(leftArmAngle)
      *leftArmAngle = (next.s.translation.x * swingMoveFadeIn - s.translation.x * swingMoveFadeOut) / p.speedMax.translation.x * p.walkArmRotation;
    if(rightArmAngle)
      *rightArmAngle = (s.translation.x * swingMoveFadeOut - next.s.translation.x * swingMoveFadeIn) / p.speedMax.translation.x * p.walkArmRotation;
  }
  else
  {
    const Vector3<> leftStepOffsetTranslation = next.l * sl - (next.s.translation + s.translation) * swingMoveFadeOut;

    Vector3<> leftStepOffsetRotation = next.lRotation * sl;
    leftStepOffsetRotation.z += next.s.rotation * swingMoveFadeIn;
    const Vector3<> rightStepOffsetRotation(0.f, 0.f, s.rotation * swingMoveFadeOut);

    stance.rightOriginToCom = refToCom + r;
    stance.rightOriginToFoot = Pose3D(RotationMatrix(rightStepOffsetRotation), Vector3<>(0.f, -p.standComPosition.y, 0.f));

    stance.leftOriginToCom = refToCom + r - next.s.translation;
    stance.leftOriginToFoot = Pose3D(RotationMatrix(leftStepOffsetRotation), Vector3<>(0.f, p.standComPosition.y, 0.f) + leftStepOffsetTranslation);

    if(rightArmAngle)
      *rightArmAngle = (next.s.translation.x * swingMoveFadeIn - s.translation.x * swingMoveFadeOut) / p.speedMax.translation.x * p.walkArmRotation;
    if(leftArmAngle)
      *leftArmAngle = (s.translation.x * swingMoveFadeOut - next.s.translation.x * swingMoveFadeIn) / p.speedMax.translation.x * p.walkArmRotation;
  }

  if(stepOffset)
  {
    stepOffset->translation.x = next.s.translation.x * swingMoveFadeIn - s.translation.x * swingMoveFadeOut;
    stepOffset->translation.y = next.s.translation.y * swingMoveFadeIn - s.translation.y * swingMoveFadeOut;
    stepOffset->translation.z = 0.f;
    stepOffset->rotation = next.s.rotation * swingMoveFadeIn - s.rotation * swingMoveFadeOut;
  }
}

float WalkingEngine::PendulumPlayer::smoothShape(float r) const
{
  switch(walkingEngine->p.walkFadeInShape)
  {
  case WalkingEngine::Parameters::sine:
    return 0.5f - cos(r * pi) * 0.5f;
  case WalkingEngine::Parameters::sqr:
    if(r > 1.f)
      r = 2.f - r;
    return r < 0.5f ? 2.f * r * r : (4.f - 2.f * r) * r - 1.f;
  default:
    assert(false);
    return 0;
  }
}
