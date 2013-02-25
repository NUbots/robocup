#ifndef WALKKICKJOB_H
#define WALKKICKJOB_H

#include "../../../Tools/Math/Vector2.h"
#include "../MotionJob.h"

class WalkKickJob : public MotionJob
{
public:
    WalkKickJob(double time, const Vector2<>& kickposition, const Vector2<>&  kicktarget);
    Vector2<> getBallPosition(){return ball_pos;}
    Vector2<> getTargetPosition(){return target_pos;}

    virtual void summaryTo(ostream& output){}
    virtual void csvTo(ostream& output){ }


private:
    Vector2<> ball_pos;
    Vector2<> target_pos;
};

#endif // WALKKICKJOB_H
