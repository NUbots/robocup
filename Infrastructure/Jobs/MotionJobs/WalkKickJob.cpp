#include "WalkKickJob.h"

WalkKickJob::WalkKickJob(double time, const Vector2<>& kickposition, const Vector2<>&  kicktarget):MotionJob(Job::MOTION_WALK_KICK)
{
    m_job_time = time;
    ball_pos = kickposition;
    target_pos = kicktarget;

}
