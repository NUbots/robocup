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

// void WalkingEngine::breakPointIndicator(){

// }

// inline float saveAsinh(float xf)
// {
//   double x = xf; // yes, we need double here
// #ifdef _MSC_VER
//   return float(log(x + sqrt(x * x + 1.)));
// #else
//   return float(asinh(x));
// #endif
// }

// inline float saveAcosh(float xf)
// {
//   //assert(xf >= 1.f);
//   double x = xf; // yes, we need double here
//   if(x < 1.)
//     return 0.000001f;
// #ifdef WIN32
//   x = log(x + sqrt(x * x - 1.));
// #else
//   x = acosh(x);
// #endif
//   if(x < 0.000001)
//     return 0.000001f;
//   return float(x);
// }

// template<typename T> ostream& operator<<(ostream& output, const Vector3<T>& v)
// {
//     output << "[";
//     output << v.x << ",";
//     output << v.y << ",";
//     output << v.z;
//     output << "]";
//     return output;
// }

// template<typename T> ostream& operator<<(ostream& output, const Vector2<T>& v)
// {
//     output << "[";
//     output << v.x << ",";
//     output << v.y;
//     output << "]";
//     return output;
// }

/*! @brief Loads the kick config files from the config directory.
           The number of kicks loaded is 2 (==kicks.size()), the other two
           kicks are derived by reflecting the left kicks.
*/
WalkingEngine::KickPlayer::KickPlayer() : kick(NULL)
{
  config_filepath = string(CONFIG_DIR) + string("/Motion/Kicks/%s.cfg");
  assert((numOfKickTypes - 1) % 2 == 0);
  for(int i = 0; i < (numOfKickTypes - 1) / 2; ++i)
  {
    char filePath[256];
    sprintf(filePath, config_filepath.c_str(), getName(KickType(i * 2 + 1)).c_str());
    string s(filePath);
#if DEBUG_NUMOTION_VERBOSITY > 2
        debug << __PRETTY_FUNCTION__ << ": loading walk-kick from file"<<s<<endl;
#endif
    kicks.push_back(WalkingEngineKick());
    bool success = kicks.back().load(filePath);
    cout << __PRETTY_FUNCTION__ << ": load success: " << success << endl;
  }
}

string  WalkingEngine::KickPlayer::getName(KickType t)
{
    switch(t){
    case none:
        return "none";
    case left:
        return "left";
    case right:
        return "right";
    case sidewardsLeft:
        return "sidewardsLeft";
    case sidewardsRight:
        return "sidewardsRight";
    }
    return "Unknown kick type.";
}

bool WalkingEngine::KickPlayer::isKickStandKick(KickType type) const
{
  bool mirrored = (type - 1) % 2 != 0;
  const WalkingEngineKick& kick = kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
  return kick.isStandKick();
}

void WalkingEngine::KickPlayer::getKickStepSize(KickType type, float& rotation, Vector3<>& translation) const
{
  bool mirrored = (type - 1) % 2 != 0;
  const WalkingEngineKick& kick = kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
  kick.getStepSize(rotation, translation);
  if(mirrored)
  {
    translation.y = -translation.y;
    rotation = -rotation;
  }
}

void WalkingEngine::KickPlayer::getKickPreStepSize(KickType type, float& rotation, Vector3<>& translation) const
{
  bool mirrored = (type - 1) % 2 != 0;
  const WalkingEngineKick& kick = kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
  kick.getPreStepSize(rotation, translation);
  if(mirrored)
  {
    translation.y = -translation.y;
    rotation = -rotation;
  }
}

float WalkingEngine::KickPlayer::getKickDuration(KickType type) const
{
  bool mirrored = (type - 1) % 2 != 0;
  const WalkingEngineKick& kick = kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
  return kick.getDuration();
}

float WalkingEngine::KickPlayer::getKickRefX(KickType type, float defaultValue) const
{
  bool mirrored = (type - 1) % 2 != 0;
  const WalkingEngineKick& kick = kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
  return kick.getRefX(defaultValue);
}

void WalkingEngine::KickPlayer::init(KickType type, const Vector2<>& ballPosition, const Vector2<>& target)
{
#if DEBUG_NUMOTION_VERBOSITY > 2
  debug << " WalkingEngine::KickPlayer::init "<<endl;
#endif
  assert(!kick);    
  mirrored = (type - 1) % 2 != 0; //If even
  this->type = type;
  kick = &kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];   
  setParameters(ballPosition, target);    
  kick->init();
}

void WalkingEngine::KickPlayer::seek(float deltaT)
{
   if(kick)
    if(!kick->seek(deltaT))
    {
#if DEBUG_NUMOTION_VERBOSITY > 2
      debug << " WalkingEngine::KickPlayer::seek():: seek failed -  - setting kick to none "<<endl;
#endif
      kick = 0;
    }
}

float WalkingEngine::KickPlayer::getLength() const
{
  if(kick)
  {
    float length = kick->getLength();
    debug <<__PRETTY_FUNCTION__ << ": length: " << length << std::endl;
    return length;
  }
  cout<<"WalkingEngine::KickPlayer::getLength() asserted false"<<endl;
  assert(false);
  return -1.f;
}

float WalkingEngine::KickPlayer::getCurrentPosition() const
{
#if DEBUG_NUMOTION_VERBOSITY > 2
        debug << "WalkingEngine::KickPlayer::getCurrentPosition() - start " << endl;
#endif

  if(kick){
#if DEBUG_NUMOTION_VERBOSITY > 2
    debug << "WalkingEngine::KickPlayer::getCurrentPosition() - start "<< kick <<endl;
#endif
    float f = kick->getCurrentPosition();
#if DEBUG_NUMOTION_VERBOSITY > 2
     debug << "WalkingEngine::KickPlayer::getCurrentPosition() - start " << f <<endl;
#endif
    return f;
  }
  cout<<"WalkingEngine::KickPlayer::getCurrentPosition() asserted false"<<endl;
  assert(false);
  return -1.f;
}

void WalkingEngine::KickPlayer::apply(Stance& stance)
{
  if(!kick)
    return;
  Vector3<> additionalFootRotation;
  Vector3<> additionFootTranslation;
  float additionHeadAngles[2];
  float additionLeftArmAngles[4];
  float additionRightArmAngles[4];

  for(int i = 0; i < 2; ++i)
    additionHeadAngles[i] = kick->getValue(WalkingEngineKick::Track(WalkingEngineKick::headYaw + i), 0.f);
  for(int i = 0; i < 4; ++i)
  {
    additionLeftArmAngles[i] = kick->getValue(WalkingEngineKick::Track(WalkingEngineKick::lShoulderPitch + i), 0.f);
    additionRightArmAngles[i] = kick->getValue(WalkingEngineKick::Track(WalkingEngineKick::rShoulderPitch + i), 0.f);
  }
  for(int i = 0; i < 3; ++i)
  {
    additionFootTranslation[i] = kick->getValue(WalkingEngineKick::Track(WalkingEngineKick::footTranslationX + i), 0.f);
    additionalFootRotation[i] = kick->getValue(WalkingEngineKick::Track(WalkingEngineKick::footRotationX + i), 0.f);
  }

  if(mirrored)
  {
    additionalFootRotation.x = -additionalFootRotation.x;
    additionalFootRotation.z = -additionalFootRotation.z;
    additionFootTranslation.y = -additionFootTranslation.y;

    for(unsigned int i = 0; i < sizeof(stance.leftArmJointAngles) / sizeof(*stance.leftArmJointAngles); ++i)
    {
      float tmp = additionLeftArmAngles[i];
      additionLeftArmAngles[i] = additionRightArmAngles[i];
      additionRightArmAngles[i] = tmp;
    }
    additionHeadAngles[0] = -additionHeadAngles[0];
  }

  (mirrored ? stance.rightOriginToFoot : stance.leftOriginToFoot).conc(Pose3D(RotationMatrix(additionalFootRotation), additionFootTranslation));
  for(int i = 0; i < 2; ++i)
    if(stance.headJointAngles[i] != 1000/*=JointData::off*/)
      stance.headJointAngles[i] += additionHeadAngles[i];
  for(int i = 0; i < 4; ++i)
  {
    stance.leftArmJointAngles[i] += additionLeftArmAngles[i];
    stance.rightArmJointAngles[i] += additionRightArmAngles[i];
  }

}

void WalkingEngine::KickPlayer::setParameters(const Vector2<>& ballPosition, const Vector2<>& target)
{
  if(!kick)
    return;
  if(mirrored)
    kick->setParameters(Vector2<>(ballPosition.x, -ballPosition.y), Vector2<>(target.x, -target.y));
  else
    kick->setParameters(ballPosition, target);
}

/*InMessage class dead
bool WalkingEngine::KickPlayer::handleMessage(InMessage& message)
{
  if(message.getMessageID() == idWalkingEngine
)
  {
    unsigned int id, size;
    message.bin >> id >> size;
    assert(id < numOfKickTypes);
    char* buffer = new char[size + 1];
    message.bin.read(buffer, size);
    buffer[size] = '\0';
    char filePath[256];
    sprintf(filePath, config_path.c_str(), getName(KickType(id)).c_str());
    if(kicks[(id - 1) / 2].load(filePath, buffer))
    {
      OUTPUT(idText, text, filePath << ": ok");
    }
    delete[] buffer;
    return true;
  }
  else
    return false;
}
*/
