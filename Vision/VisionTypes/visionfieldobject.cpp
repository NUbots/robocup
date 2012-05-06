#include "visionfieldobject.h"

//std::map<VisionFieldObject::VFO_ID, VisionID::EXTERNAL_FIELD_OBJECT_ID> createMap()
//{
//  std::map<VisionFieldObject::VFO_ID, VisionID::EXTERNAL_FIELD_OBJECT_ID> m;
//  m[VisionFieldObject::BALL]            = VisionID::BALL;
//  m[VisionFieldObject::GOAL_Y]          = VisionID::GOAL_Y;
//  m[VisionFieldObject::GOAL_B]          = VisionID::GOAL_B;
//  m[VisionFieldObject::LINE]            = VisionID::LINE;
//  m[VisionFieldObject::CORNER]          = VisionID::CORNER;
//  m[VisionFieldObject::CENTRE_CIRCLE]   = VisionID::CENTRE_CIRCLE;
//  m[VisionFieldObject::OBSTACLE]        = VisionID::OBSTACLE;
//  m[VisionFieldObject::UNKNOWN]         = VisionID::UNKNOWN;
//  return m;
//}

//static std::map<VisionFieldObject::VFO_ID, VisionID::EXTERNAL_FIELD_OBJECT_ID> id_map = createMap();

std::string VisionFieldObject::getVFOName(VFO_ID id)
{
    switch(id) {
    case BALL:          return "BALL";
    case GOAL_Y:        return "GOAL_Y";
    case GOAL_B:        return "GOAL_B";
    case LINE:          return "LINE";
    case CORNER:        return "CORNER";
    case CENTRE_CIRCLE: return "CENTRE_CIRCLE";
    case OBSTACLE:      return "OBSTACLE";
    case UNKNOWN:       return "UNKNOWN";
    }
}

VisionFieldObject::VFO_ID VisionFieldObject::getVFOFromName(const std::string &name)
{
    if(name.compare("BALL") == 0)
        return BALL;
    else if(name.compare("GOAL_Y") == 0)
        return GOAL_Y;
    else if(name.compare("GOAL_B") == 0)
        return GOAL_B;
    else if(name.compare("LINE") == 0)
        return LINE;
    else if(name.compare("CORNER") == 0)
        return CORNER;
    else if(name.compare("CENTRE_CIRCLE") == 0)
        return CENTRE_CIRCLE;
    else if(name.compare("OBSTACLE") == 0)
        return OBSTACLE;
    else
        return UNKNOWN;
}

VisionFieldObject::VisionFieldObject()
{
    
}

//VisionFieldObject::VisionFieldObject(VFO_ID id)
//{
//    this->id = id;
//}
