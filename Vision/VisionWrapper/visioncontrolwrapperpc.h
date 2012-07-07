#ifndef CONTROLWRAPPER_H
#define CONTROLWRAPPER_H

#include "Vision/visioncontroller.h"
#include "Vision/VisionWrapper/datawrapperpc.h"

class NUSensorsData;
class NUActionatorsData;

class VisionControlWrapper
{
public:
    static VisionControlWrapper* getInstance();
    
    int runFrame(bool look_for_ball);
    
private:
    VisionControlWrapper();
    
    static VisionControlWrapper* instance;
    
    VisionController* controller;
    DataWrapper* wrapper;
};

#endif // CONTROLWRAPPER_H
