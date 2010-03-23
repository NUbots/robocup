#include "GameInformation.h"
#include "NUPlatform/NUSensors.h"
#include <vector>

GameInformation::GameInformation(int playerNumber, int teamNumber):
        m_myPlayerNumber(playerNumber),
        m_myTeamNumber(teamNumber)
{
    m_myStateChanged = true;
    // Setup initial state
    // If  a player number, set to initial state.
    if((playerNumber >= 1) && (playerNumber <= 3))
    {
        m_myCurrentState = state_initial;
        m_myPreviousState = state_initial;
    }
    // If a non-player number, set to sustitute state.
    else
    {
        m_myCurrentState = state_substitute;
        m_myPreviousState = state_substitute;
    }
}

void GameInformation::update(NUSensorsData* sensorData, const RoboCupGameControlData* gameControllerPacket)
{
    if(sensorData)
    {
        updateSensorData(sensorData);
    }
    if(gameControllerPacket)
    {
        updateNetworkData(gameControllerPacket);
    }
    return;
}


void GameInformation::updateSensorData(NUSensorsData* sensorData)
{
    std::vector<float> tempButtonValues;
    std::vector<float> tempButtonTriggers;
    bool chestPressed = false;
    bool leftFootPressed = false;
    bool rightFootPressed = false;
    bool stateChangeTriggered = false;
    m_myStateChanged = false;

    if(sensorData->getButtonValues(NUSensorsData::MainButton, tempButtonValues) && tempButtonValues.size() >= 1)
    {
        chestPressed = (tempButtonValues[0] > 0.0f);
    }

    if(sensorData->getFootBumperValues(NUSensorsData::AllFeet,tempButtonValues) && tempButtonValues.size() >= 2)
    {
        leftFootPressed = (tempButtonValues[0] > 0.0f);
        rightFootPressed = (tempButtonValues[1] > 0.0f);
    }

    ;
    if(sensorData->getButtonTriggers(tempButtonTriggers) && tempButtonTriggers.size() >= 3)
    {
        if(chestPressed && (tempButtonTriggers[0] < 33.0f ))
        {
            stateChangeTriggered = true;
        }
    }

    if(stateChangeTriggered)
    {
        m_myPreviousState = m_myCurrentState;
        m_myCurrentState = getNextState(m_myCurrentState);
        m_myStateChanged = true;
    }
}

void GameInformation::updateNetworkData(const RoboCupGameControlData* gameControllerPacket)
{
    m_previousControlData = m_currentControlData;
    m_currentControlData = *gameControllerPacket;
}


robotState GameInformation::getNextState(robotState currentState)
{
    robotState nextState(currentState);
    switch(currentState)
    {
        case state_initial:
            nextState = state_ready;
            break;
        case state_ready:
            nextState = state_set;
            break;
        case state_set:
            nextState = state_playing;
            break;
        case state_playing:
            nextState = state_penalised;
            break;
        case state_finished:
            nextState = state_finished;
            break;
        case state_penalised:
            nextState = state_playing;
            break;
        case state_substitute:
            nextState = state_substitute;
            break;
        case state_requires_substitution:
            nextState = state_requires_substitution;
            break;
        default:
            nextState = currentState;
            break;
    }
    return nextState;
}
