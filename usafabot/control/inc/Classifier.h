/**
 * @file      Classifier.h
 * @brief     Provide functions that are used to Classify states of walls
 * @details   Uses wall sensors to classify state of robot
 * @version   V1.0
 * @author    Beyer
 * @copyright 
 * @warning   AS-IS
 * @note      
 * @date      November 11, 2019
 ******************************************************************************/

enum scenario {
    Error = 0,
    LeftTooClose = 1,
    RightTooClose = 2,
    CenterTooClose = 4,
    Straight = 8,
    LeftTurn = 9,
    RightTurn = 10,
    TeeJoint = 11,
    LeftJoint = 12,
    RightJoint = 13,
    CrossRoad = 14,
    Blocked = 15
};
typedef enum scenario scenario_t;

scenario_t Classify(int32_t Left, int32_t Center, int32_t Right);

int32_t Convert(int32_t n);
