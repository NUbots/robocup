/*!  @file ScriptTunerStates.h
    @brief Provider of Head behaviour Testing behaviour. Darwin simply stands, observes and localises. Modified from Zombie state.
    @author Jake Fountain

    @author Jake Fountain

    Copyright (c) 2012 Jake Fountain

    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SCRIPT_TUNER_STATES_H
#define SCRIPT_TUNER_STATES_H

#include "Behaviour/BehaviourState.h"
#include "ScriptTunerProvider.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"
#include "Infrastructure/Jobs/MotionJobs/MotionFreezeJob.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include <thread>
#include <chrono>
#include "Framework/darwin/Framework/include/JointData.h"
#include "Motion/Kicks/MotionScript2013.h"
#include "Infrastructure/NUData.h"
#include "NUPlatform/NUPlatform.h"
    
#include "Autoconfig/nubotdataconfig.h"

#include "Behaviour/Common/HeadBehaviour.h"


#include "debug.h"

using std::vector;
using std::string;

class ScriptTunerCommand
{
public:
    enum class CommandType {
        kUnknown,           //!< Unknown command.
        kHelp,              //!< Prints help for the ScriptTuner.
        kExit,              //!< Exit the script tuner.
        kPlay,              //!< Plays the current script.
        kEdit,              //!< Begin editing the current sctipr.
        kNextFrame,         //!< Move to the next frame.
        kPreviousFrame,     //!< Move to the previous frame.
        kNewFrame,          //!< Creates a new frame after the current frame.
        kFrameSeek,         //!< Seek to a given frame.
        kFrameDuration,     //!< Set the duration of the current frame.
        kAllOn,             //!< Turn on all motors.
        kJointPosition,     //!< Modify the position of a given joint.
        kJointGain,         //!< Modify the gain of a given joint.
        kSetAllGains,       //!< Set the gain of all joints to the same value.
        kJointPositionGain, //!< Modify the gain of a given joint.
        kJointOff,          //!< Temporarily turn off a given joint, for editing.
        kJointOn,           //!< Turn on a given joint.
        kSaveFrame,         //!< Save the current frame.
        kSaveScript,        //!< Save the current script.
        kLoadScript,        //!< Load a script from the specified path in the config system.
        kLoadOldScript,     //!< Load an old script from the given path.
        kPrintScript,       //!< Prints a text representation of the script.
        kPrint,             //!< Prints the current frame.
        kAllOff,            //!< Turns all torques off for manual editing.
        kSetPose,           //!< Turns torque to low value 
        kDeleteFrame,       //!< Deletes current frame
    };

    static int MapJointInitialismToServoId(const std::string& initials)
    {
        if(!initials.compare("RSP")) return Robot::JointData::ID_R_SHOULDER_PITCH;
        if(!initials.compare("LSP")) return Robot::JointData::ID_L_SHOULDER_PITCH;
        if(!initials.compare("RSR")) return Robot::JointData::ID_R_SHOULDER_ROLL;
        if(!initials.compare("LSR")) return Robot::JointData::ID_L_SHOULDER_ROLL;
        if(!initials.compare("RE" )) return Robot::JointData::ID_R_ELBOW;
        if(!initials.compare("LE" )) return Robot::JointData::ID_L_ELBOW;
        if(!initials.compare("RHY")) return Robot::JointData::ID_R_HIP_YAW;
        if(!initials.compare("LHY")) return Robot::JointData::ID_L_HIP_YAW;
        if(!initials.compare("RHR")) return Robot::JointData::ID_R_HIP_ROLL;
        if(!initials.compare("LHR")) return Robot::JointData::ID_L_HIP_ROLL;
        if(!initials.compare("RHP")) return Robot::JointData::ID_R_HIP_PITCH;
        if(!initials.compare("LHP")) return Robot::JointData::ID_L_HIP_PITCH;
        if(!initials.compare("RK" )) return Robot::JointData::ID_R_KNEE;
        if(!initials.compare("LK" )) return Robot::JointData::ID_L_KNEE;
        if(!initials.compare("RAP")) return Robot::JointData::ID_R_ANKLE_PITCH;
        if(!initials.compare("LAP")) return Robot::JointData::ID_L_ANKLE_PITCH;
        if(!initials.compare("RAR")) return Robot::JointData::ID_R_ANKLE_ROLL;
        if(!initials.compare("LAR")) return Robot::JointData::ID_L_ANKLE_ROLL;
        if(!initials.compare("HP" )) return Robot::JointData::ID_HEAD_PAN;
        if(!initials.compare("HT" )) return Robot::JointData::ID_HEAD_TILT;
        return Robot::JointData::ID_UNKNOWN;
    }

    static ScriptTunerCommand ParseCommand(const std::string& command)
    {
        std::stringstream command_ss;
        command_ss << command;

        std::string arg0;
        command_ss >> arg0;

        if(!arg0.compare("help")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kHelp);
            return c;
        } else if(!arg0.compare("printscript")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kPrintScript);
            return c;
        } else if(!arg0.compare("load")) {
            ScriptTunerCommand c;
            std::string script_path;
            if(!(command_ss >> script_path))
                return c;
            c.set_command_type(CommandType::kLoadScript);
            c.set_script_path(script_path);
            return c;
        } else if(!arg0.compare("loadold")) {
            ScriptTunerCommand c;
            std::string script_path;
            if(!(command_ss >> script_path))
                return c;
            c.set_command_type(CommandType::kLoadOldScript);
            c.set_script_path(script_path);
            return c;
        } else if(!arg0.compare("play")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kPlay);
            return c;
        } else if(!arg0.compare("edit")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kEdit);
            return c;
        } else if(!arg0.compare("exit")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kExit);
            return c;
        } else if(!arg0.compare("quit")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kExit);
            return c;
        } else if(!arg0.compare("saveframe")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kSaveFrame);
            return c;
        } else if(!arg0.compare("savescript")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kSaveScript);
            return c;
        } else if(!arg0.compare("newframe")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kNewFrame);
            return c;
        } else if(!arg0.compare("seek")) {
            ScriptTunerCommand c;
            int frame_number;
            if(!(command_ss >> frame_number))
                return c;
            c.set_command_type(CommandType::kFrameSeek);
            c.set_frame_number(frame_number-1);
            return c;
        } else if(!arg0.compare("duration")) {
            ScriptTunerCommand c;
            float duration;
            if(!(command_ss >> duration))
                return c;
            c.set_command_type(CommandType::kFrameDuration);
            c.set_duration(duration);
            return c;
        } else if(!arg0.compare("allon")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kAllOn);
            return c;
        } else if(!arg0.compare("alloff")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kAllOff);
            return c;
        } else if(!arg0.compare("on")) {
            ScriptTunerCommand c;
            std::string joint_initials;
            if(!(command_ss >> joint_initials))
                return c;
            int joint_id = MapJointInitialismToServoId(joint_initials);
            if(joint_id==Robot::JointData::ID_UNKNOWN) 
                return c;
            c.set_command_type(CommandType::kJointOn);
            c.set_joint_number(joint_id);
            return c;
        } else if(!arg0.compare("off")) {
            ScriptTunerCommand c;
            std::string joint_initials;
            if(!(command_ss >> joint_initials))
                return c;
            int joint_id = MapJointInitialismToServoId(joint_initials);
            if(joint_id==Robot::JointData::ID_UNKNOWN) 
                return c;
            c.set_command_type(CommandType::kJointOff);
            c.set_joint_number(joint_id);
            return c;
        } else if(!arg0.compare("next")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kNextFrame);
            return c;
        } else if(!arg0.compare("prev") || !arg0.compare("previous")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kPreviousFrame);
            return c; 
        } else if (!arg0.compare("setpose")){
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kSetPose);
            return c;
        } else if (!arg0.compare("delete")){
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kDeleteFrame);
            return c;
        } else if (!arg0.compare("print")){
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kPrint);
            return c;
        } else if (!arg0.compare("gain")){
            ScriptTunerCommand c;
            float gain;
            if((command_ss >> gain))
                return c;
            int joint_id = MapJointInitialismToServoId(arg0);
            c.set_command_type(CommandType::kJointGain);
            c.set_joint_number(joint_id);
            c.set_gain(gain);
            return c;
        } else if (!arg0.compare("setgains")){
            ScriptTunerCommand c;
            float gain;
            if((command_ss >> gain))
                return c;
            int joint_id = MapJointInitialismToServoId(arg0);
            c.set_command_type(CommandType::kSetAllGains);
            c.set_gain(gain);
            return c;
        } else {
            ScriptTunerCommand c;
            float position;
            float gain;
            bool set_gain = false;
            if(!(command_ss >> position))
                return c;
            if((command_ss >> gain))
                set_gain = true;
            int joint_id = MapJointInitialismToServoId(arg0);
            c.set_command_type(CommandType::kJointPositionGain);
            c.set_joint_number(joint_id);
            c.set_position(position);
            if(set_gain)
                c.set_gain(gain);
            return c;
        }
    }

    static void PrintCommandsLongHelp()
    {
        std::cout << "Command List While Editing:" << std::endl;
        std::cout << "next" << std::endl;
        std::cout << "delete" << std::endl;
        std::cout << "<motor name> off" << std::endl;
        std::cout << "<motor name> on" << std::endl;
        std::cout << "allon" << std::endl;
        std::cout << "duration <duration value in ms>" << std::endl;
        std::cout << "seek <frame number>" << std::endl;
        std::cout << "play"<< std::endl;
        std::cout << "edit"<< std::endl;
        std::cout << "exit"<< std::endl;
        std::cout << "setpose"<< std::endl;
        std::cout << "quit"<< std::endl;
        std::cout << "saveframe"<< std::endl;
        std::cout << "savescript"<< std::endl;
        std::cout << "newframe <duration>"<< std::endl;
        std::cout << std::endl;
        std::cout << "Script Tuner Command Help:" << std::endl;
        std::cout << std::endl;
        std::cout << "Note: When a command requires a joint to be specified, " << std::endl;
        std::cout << "      it should be denoted using one of the following initialisms:" << std::endl;
        std::cout << std::endl;
        std::cout << "RSP = Right Shoulder Pitch;" << std::endl;
        std::cout << "LSP = Left Shoulder Pitch;" << std::endl;
        std::cout << "RSR = Right Shoulder Roll;" << std::endl;
        std::cout << "LSR = Left Shoulder Roll;" << std::endl;
        std::cout << "RE  = Right Elbow;" << std::endl;
        std::cout << "LE  = Left Elbow;" << std::endl;
        std::cout << "RHY = Right Hip Yaw;" << std::endl;
        std::cout << "LHY = Left Hip Yaw;" << std::endl;
        std::cout << "RHR = Right Hip Roll;" << std::endl;
        std::cout << "LHR = Left Hip Roll;" << std::endl;
        std::cout << "RHP = Right Hip Pitch;" << std::endl;
        std::cout << "LHP = Left Hip Pitch;" << std::endl;
        std::cout << "RK  = Right Knee;" << std::endl;
        std::cout << "LK  = Left Knee;" << std::endl;
        std::cout << "RAP = Right Ankle Pitch;" << std::endl;
        std::cout << "LAP = Left Ankle Pitch;" << std::endl;
        std::cout << "RAR = Right Ankle Roll;" << std::endl;
        std::cout << "LAR = Left Ankle Roll;" << std::endl;
        std::cout << "HP  = Head Pan;" << std::endl;
        std::cout << "HT  = Head Tilt;" << std::endl;
    }

    ScriptTunerCommand() : command_type_(CommandType::kUnknown),
                           position_(0),
                           gain_(0),
                           duration_(1000),
                           disable_(false),
                           joint_number_(0),
                           frame_number_(0) {}

    CommandType command_type() { return command_type_; }
    float position() { return position_; }
    float gain() { return gain_; }
    float duration() { return duration_; }
    bool disable() { return disable_; }
    int joint_number() { return joint_number_; }
    int frame_number() { return frame_number_; }
    std::string script_path() { return script_path_; }

    void set_command_type(CommandType command_type) { command_type_ = command_type; }
    void set_position(float position) { position_ = position; }
    void set_gain(float gain) { gain_ = gain; }
    void set_duration(float duration) { duration_ = duration; }
    void set_disable(bool disable) { disable_ = disable; }
    void set_joint_number(int joint_number) { joint_number_ = joint_number; }
    void set_frame_number(int frame_number) { frame_number_ = frame_number; }
    void set_script_path(std::string script_path) { script_path_ = script_path; }

private:
    CommandType command_type_;
    float position_;
    float gain_;
    float duration_;
    bool disable_;
    int joint_number_;
    int frame_number_;
    std::string script_path_;
};

class ScriptTunerSubState : public BehaviourState
{
public:
    ScriptTunerSubState(ScriptTunerProvider* provider){m_provider = provider;}
protected:
    ScriptTunerProvider* m_provider;
};

// ----------------------------------------------------------------------------------------------------------------------- PausedState
class ScriptTunerState : public ScriptTunerSubState
{
private:
    const float kSetPoseCommandGains;

    void HandleDeleteFrameCommand(ScriptTunerCommand command);
    void HandleSetPoseCommand(ScriptTunerCommand command);
    void HandlePrintScriptCommand(ScriptTunerCommand command);
    void HandlePrintFrameCommand(ScriptTunerCommand command);
    void HandleLoadScriptCommand(ScriptTunerCommand command);
    void HandleLoadOldScriptCommand(ScriptTunerCommand command);
    void HandleEditCommand(ScriptTunerCommand command);
    void HandlePlayCommand(ScriptTunerCommand command);
    void HandleSaveFrameCommand(ScriptTunerCommand command);
    void HandleSaveScriptCommand(ScriptTunerCommand command);
    void HandleExitScriptCommand(ScriptTunerCommand command);
    void HandleNextFrameCommand(ScriptTunerCommand command);
    void HandlePreviousFrameCommand(ScriptTunerCommand command);
    void HandleNewFrameCommand(ScriptTunerCommand command);
    void HandleFrameSeekCommand(ScriptTunerCommand command);
    void HandleFrameDurationCommand(ScriptTunerCommand command);
    void HandleAllOnCommand(ScriptTunerCommand command);
    void HandleJointOffCommand(ScriptTunerCommand command);
    void HandleJointOnCommand(ScriptTunerCommand command);
    void HandleJointPositionCommand(ScriptTunerCommand command);
    void HandleJointGainCommand(ScriptTunerCommand command);
    void HandleSetAllGainsCommand(ScriptTunerCommand command);
    void HandleJointPositionGainCommand(ScriptTunerCommand command);
    void PrintCommandError(ScriptTunerCommand command);
    void HandleAllOffCommand(ScriptTunerCommand command);


    
    std::string file_name_;
    std::string file_path_;

    MotionScript2013* script_;

    bool script_active_;

    // Contains motors which have had torque turned off and then on.
    std::vector<int> motors_to_be_saved_;

    NUActionatorsData* actionators_data_;
    NUSensorsData* sensors_data_;

public:
    ScriptTunerState(ScriptTunerProvider* provider);

    BehaviourState* nextState() {return m_provider->m_state;}
    void doState();

    void HandleHelpCommand(ScriptTunerCommand command);

    void PrintFrameInfo();

    void editCurrentFrame();

    /*! @brief Loads a motion script from a file into the member variables frames and times */
    bool loadScript(std::string filename);

     /*! @brief Reads motor positions from robot and saves them to the current frame, frames[frame].
        Only save motor positions which have had torque off and then back on.*/
    void saveManuallyMovedMotors();

     /*! @brief Writes the frames to script file.*/
    bool saveScriptToFile(std::string filename);

    void UpdateScriptMotorGoalsToCurrentPosition(MotionScriptFrame* frame);


    void CopyPositionsFromTempScript(MotionScriptFrame* temp_frame);
    
    /*! @brief Motor fine tuning.
        @param pos_change is change in angle in radians.*/
    void changeMotorPosition(int motor_id,float pos_change);
    
    /*! @brief Motor fine tuning.
        @param gain_change is change in gain.*/    
    void changeMotorGain(int motor_id, float gain_change);
    
    /*! @brief Motor turn off torque method.*/    
    void turnOffMotor(int motor_id);
    
    /*! @brief Motor turn on torque method.
         Also marks motor for saving with save manually moved motors.
         Adds motion job for motor at current position with gain given in script.*/    
    void turnOnMotor(int motor_id);
    
    /*! @brief Returns the MEASURED motor position.
    */
    float getMotorPosition(int motor_id);

    /*! @brief Tests whether the motor torque is on.
    */
    bool motorTorqueIsOff(int motor_id);

    /*! @brief Turns on all motors and marks those without torque for saving.
    */
    void turnOnAllMotors();

    /*! @brief Turns of all motors.
    */
    void turnOffAllMotors();



};

#endif

