/** @file
 *	@brief MAVLink comm protocol generated from slugs.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace slugs {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (trought @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 184> MESSAGE_ENTRIES {{ {0, 50, 9, 0, 0, 0}, {1, 124, 31, 0, 0, 0}, {2, 137, 12, 0, 0, 0}, {4, 237, 14, 3, 12, 13}, {5, 217, 28, 1, 0, 0}, {6, 104, 3, 0, 0, 0}, {7, 119, 32, 0, 0, 0}, {11, 89, 6, 1, 4, 0}, {20, 214, 20, 3, 2, 3}, {21, 159, 2, 3, 0, 1}, {22, 220, 25, 0, 0, 0}, {23, 168, 23, 3, 4, 5}, {24, 24, 30, 0, 0, 0}, {25, 23, 101, 0, 0, 0}, {26, 170, 22, 0, 0, 0}, {27, 144, 26, 0, 0, 0}, {28, 67, 16, 0, 0, 0}, {29, 115, 14, 0, 0, 0}, {30, 39, 28, 0, 0, 0}, {31, 246, 32, 0, 0, 0}, {32, 185, 28, 0, 0, 0}, {33, 104, 28, 0, 0, 0}, {34, 237, 22, 0, 0, 0}, {35, 244, 22, 0, 0, 0}, {36, 222, 21, 0, 0, 0}, {37, 212, 6, 3, 4, 5}, {38, 9, 6, 3, 4, 5}, {39, 254, 37, 3, 32, 33}, {40, 230, 4, 3, 2, 3}, {41, 28, 4, 3, 2, 3}, {42, 28, 2, 0, 0, 0}, {43, 132, 2, 3, 0, 1}, {44, 221, 4, 3, 2, 3}, {45, 232, 2, 3, 0, 1}, {46, 11, 2, 0, 0, 0}, {47, 153, 3, 3, 0, 1}, {48, 41, 13, 1, 12, 0}, {49, 39, 12, 0, 0, 0}, {50, 78, 37, 3, 18, 19}, {51, 196, 4, 3, 2, 3}, {54, 15, 27, 3, 24, 25}, {55, 3, 25, 0, 0, 0}, {61, 167, 72, 0, 0, 0}, {62, 183, 26, 0, 0, 0}, {63, 119, 181, 0, 0, 0}, {64, 191, 225, 0, 0, 0}, {65, 118, 42, 0, 0, 0}, {66, 148, 6, 3, 2, 3}, {67, 21, 4, 0, 0, 0}, {69, 243, 11, 0, 0, 0}, {70, 124, 18, 3, 16, 17}, {73, 38, 37, 3, 32, 33}, {74, 20, 20, 0, 0, 0}, {75, 158, 35, 3, 30, 31}, {76, 152, 33, 3, 30, 31}, {77, 143, 3, 3, 8, 9}, {81, 106, 22, 0, 0, 0}, {82, 49, 39, 3, 36, 37}, {83, 22, 37, 0, 0, 0}, {84, 143, 53, 3, 50, 51}, {85, 140, 51, 0, 0, 0}, {86, 5, 53, 3, 50, 51}, {87, 150, 51, 0, 0, 0}, {89, 231, 28, 0, 0, 0}, {90, 183, 56, 0, 0, 0}, {91, 63, 42, 0, 0, 0}, {92, 54, 33, 0, 0, 0}, {93, 47, 81, 0, 0, 0}, {100, 175, 26, 0, 0, 0}, {101, 102, 32, 0, 0, 0}, {102, 158, 32, 0, 0, 0}, {103, 208, 20, 0, 0, 0}, {104, 56, 32, 0, 0, 0}, {105, 93, 62, 0, 0, 0}, {106, 138, 44, 0, 0, 0}, {107, 108, 64, 0, 0, 0}, {108, 32, 84, 0, 0, 0}, {109, 185, 9, 0, 0, 0}, {110, 84, 254, 3, 1, 2}, {111, 34, 16, 0, 0, 0}, {112, 174, 12, 0, 0, 0}, {113, 124, 36, 0, 0, 0}, {114, 237, 44, 0, 0, 0}, {115, 4, 64, 0, 0, 0}, {116, 76, 22, 0, 0, 0}, {117, 128, 6, 3, 4, 5}, {118, 56, 14, 0, 0, 0}, {119, 116, 12, 3, 10, 11}, {120, 134, 97, 0, 0, 0}, {121, 237, 2, 3, 0, 1}, {122, 203, 2, 3, 0, 1}, {123, 250, 113, 3, 0, 1}, {124, 87, 35, 0, 0, 0}, {125, 203, 6, 0, 0, 0}, {126, 220, 79, 0, 0, 0}, {127, 25, 35, 0, 0, 0}, {128, 226, 35, 0, 0, 0}, {129, 46, 22, 0, 0, 0}, {130, 29, 13, 0, 0, 0}, {131, 223, 255, 0, 0, 0}, {132, 85, 14, 0, 0, 0}, {133, 6, 18, 0, 0, 0}, {134, 229, 43, 0, 0, 0}, {135, 203, 8, 0, 0, 0}, {136, 1, 22, 0, 0, 0}, {137, 195, 14, 0, 0, 0}, {138, 109, 36, 0, 0, 0}, {139, 168, 43, 3, 41, 42}, {140, 181, 41, 0, 0, 0}, {141, 47, 32, 0, 0, 0}, {142, 72, 243, 0, 0, 0}, {143, 131, 14, 0, 0, 0}, {144, 127, 93, 0, 0, 0}, {146, 103, 100, 0, 0, 0}, {147, 154, 36, 0, 0, 0}, {148, 178, 60, 0, 0, 0}, {149, 200, 30, 0, 0, 0}, {170, 75, 4, 0, 0, 0}, {172, 168, 24, 0, 0, 0}, {173, 2, 18, 0, 0, 0}, {176, 228, 32, 0, 0, 0}, {177, 167, 24, 0, 0, 0}, {179, 132, 12, 0, 0, 0}, {180, 146, 13, 0, 0, 0}, {181, 104, 3, 0, 0, 0}, {184, 45, 5, 0, 0, 0}, {185, 113, 10, 0, 0, 0}, {186, 101, 9, 0, 0, 0}, {188, 5, 3, 0, 0, 0}, {189, 246, 16, 0, 0, 0}, {191, 17, 5, 0, 0, 0}, {192, 187, 5, 0, 0, 0}, {193, 160, 21, 0, 0, 0}, {194, 51, 11, 0, 0, 0}, {195, 59, 14, 0, 0, 0}, {196, 129, 11, 0, 0, 0}, {197, 39, 4, 0, 0, 0}, {230, 163, 42, 0, 0, 0}, {231, 105, 40, 0, 0, 0}, {232, 151, 63, 0, 0, 0}, {233, 35, 182, 0, 0, 0}, {234, 150, 40, 0, 0, 0}, {235, 179, 42, 0, 0, 0}, {241, 90, 32, 0, 0, 0}, {242, 104, 52, 0, 0, 0}, {243, 85, 53, 1, 52, 0}, {244, 95, 6, 0, 0, 0}, {245, 130, 2, 0, 0, 0}, {246, 184, 38, 0, 0, 0}, {247, 81, 19, 0, 0, 0}, {248, 8, 254, 3, 3, 4}, {249, 204, 36, 0, 0, 0}, {250, 49, 30, 0, 0, 0}, {251, 170, 18, 0, 0, 0}, {252, 44, 18, 0, 0, 0}, {253, 83, 51, 0, 0, 0}, {254, 46, 9, 0, 0, 0}, {256, 71, 42, 3, 8, 9}, {257, 131, 9, 0, 0, 0}, {258, 187, 32, 3, 0, 1}, {259, 92, 235, 0, 0, 0}, {260, 146, 5, 0, 0, 0}, {261, 179, 27, 0, 0, 0}, {262, 12, 18, 0, 0, 0}, {263, 133, 255, 0, 0, 0}, {264, 49, 28, 0, 0, 0}, {265, 26, 16, 0, 0, 0}, {266, 193, 255, 3, 2, 3}, {267, 35, 255, 3, 2, 3}, {268, 14, 4, 3, 2, 3}, {269, 58, 246, 0, 0, 0}, {270, 232, 247, 3, 14, 15}, {299, 19, 96, 0, 0, 0}, {300, 217, 22, 0, 0, 0}, {310, 28, 17, 0, 0, 0}, {311, 95, 116, 0, 0, 0}, {320, 243, 20, 3, 2, 3}, {321, 88, 2, 3, 0, 1}, {322, 243, 149, 0, 0, 0}, {323, 78, 147, 3, 0, 1}, {324, 132, 146, 0, 0, 0}, {330, 23, 158, 0, 0, 0}, {331, 58, 230, 0, 0, 0}, {332, 131, 234, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 2;


// ENUM DEFINITIONS


/** @brief Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. */
enum class MAV_CMD
{
    NAV_WAYPOINT=16, /* Navigate to waypoint. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at waypoint (rotary wing). NaN for unchanged.| Latitude| Longitude| Altitude|  */
    NAV_LOITER_UNLIM=17, /* Loiter around this waypoint an unlimited amount of time |Empty| Empty| Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
    NAV_LOITER_TURNS=18, /* Loiter around this waypoint for X turns |Turns| Empty| Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  */
    NAV_LOITER_TIME=19, /* Loiter around this waypoint for X seconds |Seconds (decimal)| Empty| Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  */
    NAV_RETURN_TO_LAUNCH=20, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    NAV_LAND=21, /* Land at location |Abort Alt| Precision land mode. (0 = normal landing, 1 = opportunistic precision landing, 2 = required precsion landing)| Empty| Desired yaw angle. NaN for unchanged.| Latitude| Longitude| Altitude (ground level)|  */
    NAV_TAKEOFF=22, /* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.| Latitude| Longitude| Altitude|  */
    NAV_LAND_LOCAL=23, /* Land at local position (local frame only) |Landing target number (if available)| Maximum accepted offset from desired landing position [m] - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land| Landing descend rate [ms^-1]| Desired yaw angle [rad]| Y-axis position [m]| X-axis position [m]| Z-axis / ground level position [m]|  */
    NAV_TAKEOFF_LOCAL=24, /* Takeoff from local position (local frame only) |Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]| Empty| Takeoff ascend rate [ms^-1]| Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these| Y-axis position [m]| X-axis position [m]| Z-axis position [m]|  */
    NAV_FOLLOW=25, /* Vehicle following, i.e. this waypoint represents the position of a moving vehicle |Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation| Ground speed of vehicle to be followed| Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
    NAV_CONTINUE_AND_CHANGE_ALT=30, /* Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. |Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude. | Empty| Empty| Empty| Empty| Empty| Desired altitude in meters|  */
    NAV_LOITER_TO_ALT=31, /* Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint.  |Heading Required (0 = False)| Radius in meters. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.| Empty| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location| Latitude| Longitude| Altitude|  */
    DO_FOLLOW=32, /* Being following a target |System ID (the system ID of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode| RESERVED| RESERVED| altitude flag: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home| altitude| RESERVED| TTL in seconds in which the MAV should go to the default position hold mode after a message rx timeout|  */
    DO_FOLLOW_REPOSITION=33, /* Reposition the MAV after a follow target command has been sent |Camera q1 (where 0 is on the ray from the camera to the tracking device)| Camera q2| Camera q3| Camera q4| altitude offset from target (m)| X offset from target (m)| Y offset from target (m)|  */
    NAV_ROI=80, /* THIS INTERFACE IS DEPRECATED AS OF JANUARY 2018. Please use MAV_CMD_DO_SET_ROI_* messages instead. Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
    NAV_PATHPLANNING=81, /* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
    NAV_SPLINE_WAYPOINT=82, /* Navigate to waypoint using a spline path. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)| Empty| Empty| Empty| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
    NAV_VTOL_TAKEOFF=84, /* Takeoff from ground using VTOL mode |Empty| Front transition heading, see VTOL_TRANSITION_HEADING enum.| Empty| Yaw angle in degrees. NaN for unchanged.| Latitude| Longitude| Altitude|  */
    NAV_VTOL_LAND=85, /* Land using VTOL mode |Empty| Empty| Approach altitude (with the same reference as the Altitude field). NaN if unspecified.| Yaw angle in degrees. NaN for unchanged.| Latitude| Longitude| Altitude (ground level)|  */
    NAV_GUIDED_ENABLE=92, /* hand control over to an external controller |On / Off (> 0.5f on)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    NAV_DELAY=93, /* Delay the next navigation command a number of seconds or until a specified time |Delay in seconds (decimal, -1 to enable time-of-day fields)| hour (24h format, UTC, -1 to ignore)| minute (24h format, UTC, -1 to ignore)| second (24h format, UTC)| Empty| Empty| Empty|  */
    NAV_PAYLOAD_PLACE=94, /* Descend and place payload.  Vehicle descends until it detects a hanging payload has reached the ground, the gripper is opened to release the payload |Maximum distance to descend (meters)| Empty| Empty| Empty| Latitude (deg * 1E7)| Longitude (deg * 1E7)| Altitude (meters)|  */
    NAV_LAST=95, /* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    CONDITION_DELAY=112, /* Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    CONDITION_CHANGE_ALT=113, /* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  */
    CONDITION_DISTANCE=114, /* Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    CONDITION_YAW=115, /* Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  */
    CONDITION_LAST=159, /* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_SET_MODE=176, /* Set system mode. |Mode, as defined by ENUM MAV_MODE| Custom mode - this is system specific, please refer to the individual autopilot specifications for details.| Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.| Empty| Empty| Empty| Empty|  */
    DO_JUMP=177, /* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  */
    DO_CHANGE_SPEED=178, /* Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| absolute or relative [0,1]| Empty| Empty| Empty|  */
    DO_SET_HOME=179, /* Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
    DO_SET_PARAMETER=180, /* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */
    DO_SET_RELAY=181, /* Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  */
    DO_REPEAT_RELAY=182, /* Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  */
    DO_SET_SERVO=183, /* Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  */
    DO_REPEAT_SERVO=184, /* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  */
    DO_FLIGHTTERMINATION=185, /* Terminate flight immediately |Flight termination activated if > 0.5| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_CHANGE_ALTITUDE=186, /* Change altitude set point. |Altitude in meters| Mav frame of new altitude (see MAV_FRAME)| Empty| Empty| Empty| Empty| Empty|  */
    DO_LAND_START=189, /* Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence. |Empty| Empty| Empty| Empty| Latitude| Longitude| Empty|  */
    DO_RALLY_LAND=190, /* Mission command to perform a landing from a rally point. |Break altitude (meters)| Landing speed (m/s)| Empty| Empty| Empty| Empty| Empty|  */
    DO_GO_AROUND=191, /* Mission command to safely abort an autonmous landing. |Altitude (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_REPOSITION=192, /* Reposition the vehicle to a specific WGS84 global position. |Ground speed, less than 0 (-1) for default| Bitmask of option flags, see the MAV_DO_REPOSITION_FLAGS enum.| Reserved| Yaw heading, NaN for unchanged. For planes indicates loiter direction (0: clockwise, 1: counter clockwise)| Latitude (deg * 1E7)| Longitude (deg * 1E7)| Altitude (meters)|  */
    DO_PAUSE_CONTINUE=193, /* If in a GPS controlled position mode, hold the current position or continue. |0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    DO_SET_REVERSE=194, /* Set moving direction to forward or reverse. |Direction (0=Forward, 1=Reverse)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_SET_ROI_LOCATION=195, /* Sets the region of interest (ROI) to a location. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Empty| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
    DO_SET_ROI_WPNEXT_OFFSET=196, /* Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Empty| Empty| Empty| Empty| pitch offset from next waypoint| roll offset from next waypoint| yaw offset from next waypoint|  */
    DO_SET_ROI_NONE=197, /* Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_CONTROL_VIDEO=200, /* Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */
    DO_SET_ROI=201, /* THIS INTERFACE IS DEPRECATED AS OF JANUARY 2018. Please use MAV_CMD_DO_SET_ROI_* messages instead. Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| MAV_ROI_WPNEXT: pitch offset from next waypoint, MAV_ROI_LOCATION: latitude| MAV_ROI_WPNEXT: roll offset from next waypoint, MAV_ROI_LOCATION: longitude| MAV_ROI_WPNEXT: yaw offset from next waypoint, MAV_ROI_LOCATION: altitude|  */
    DO_DIGICAM_CONFIGURE=202, /* THIS INTERFACE IS DEPRECATED since 2018-01. Please use PARAM_EXT_XXX messages and the camera definition format described in https://mavlink.io/en/protocol/camera_def.html. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|  */
    DO_DIGICAM_CONTROL=203, /* THIS INTERFACE IS DEPRECATED since 2018-01. Please use PARAM_EXT_XXX messages and the camera definition format described in https://mavlink.io/en/protocol/camera_def.html. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.|  */
    DO_MOUNT_CONFIGURE=204, /* Mission command to configure a camera or antenna mount |Mount operation mode (see MAV_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| roll input (0 = angle, 1 = angular rate)| pitch input (0 = angle, 1 = angular rate)| yaw input (0 = angle, 1 = angular rate)|  */
    DO_MOUNT_CONTROL=205, /* Mission command to control a camera or antenna mount |pitch depending on mount mode (degrees or degrees/second depending on pitch input).| roll depending on mount mode (degrees or degrees/second depending on roll input).| yaw depending on mount mode (degrees or degrees/second depending on yaw input).| alt in meters depending on mount mode.| latitude in degrees * 1E7, set if appropriate mount mode.| longitude in degrees * 1E7, set if appropriate mount mode.| MAV_MOUNT_MODE enum value|  */
    DO_SET_CAM_TRIGG_DIST=206, /* Mission command to set camera trigger distance for this flight. The camera is trigerred each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera. |Camera trigger distance (meters). 0 to stop triggering.| Camera shutter integration time (milliseconds). -1 or 0 to ignore| Trigger camera once immediately. (0 = no trigger, 1 = trigger)| Empty| Empty| Empty| Empty|  */
    DO_FENCE_ENABLE=207, /* Mission command to enable the geofence |enable? (0=disable, 1=enable, 2=disable_floor_only)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_PARACHUTE=208, /* Mission command to trigger a parachute |action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_MOTOR_TEST=209, /* Mission command to perform motor test |motor number (a number from 1 to max number of motors on the vehicle)| throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)| throttle| timeout (in seconds)| motor count (number of motors to test to test in sequence, waiting for the timeout above between them; 0=1 motor, 1=1 motor, 2=2 motors...)| motor test order (See MOTOR_TEST_ORDER enum)| Empty|  */
    DO_INVERTED_FLIGHT=210, /* Change to/from inverted flight |inverted (0=normal, 1=inverted)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    NAV_SET_YAW_SPEED=213, /* Sets a desired vehicle turn angle and speed change |yaw angle to adjust steering by in centidegress| speed - normalized to 0 .. 1| Empty| Empty| Empty| Empty| Empty|  */
    DO_SET_CAM_TRIGG_INTERVAL=214, /* Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera. |Camera trigger cycle time (milliseconds). -1 or 0 to ignore.| Camera shutter integration time (milliseconds). Should be less than trigger cycle time. -1 or 0 to ignore.| Empty| Empty| Empty| Empty| Empty|  */
    DO_MOUNT_CONTROL_QUAT=220, /* Mission command to control a camera or antenna mount, using a quaternion as reference. |q1 - quaternion param #1, w (1 in null-rotation)| q2 - quaternion param #2, x (0 in null-rotation)| q3 - quaternion param #3, y (0 in null-rotation)| q4 - quaternion param #4, z (0 in null-rotation)| Empty| Empty| Empty|  */
    DO_GUIDED_MASTER=221, /* set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|  */
    DO_GUIDED_LIMITS=222, /* set limits for external control |timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout| absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit| absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit| horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit| Empty| Empty| Empty|  */
    DO_ENGINE_CONTROL=223, /* Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines |0: Stop engine, 1:Start Engine| 0: Warm start, 1:Cold start. Controls use of choke where applicable| Height delay (meters). This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.| Empty| Empty| Empty| Empty| Empty|  */
    DO_LAST=240, /* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    PREFLIGHT_CALIBRATION=241, /* Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero. |1: gyro calibration, 3: gyro temperature calibration| 1: magnetometer calibration| 1: ground pressure calibration| 1: radio RC calibration, 2: RC trim calibration| 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration| 1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration| 1: ESC calibration, 3: barometer temperature calibration|  */
    PREFLIGHT_SET_SENSOR_OFFSETS=242, /* Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  */
    PREFLIGHT_UAVCAN=243, /* Trigger UAVCAN config. This command will be only accepted if in pre-flight mode. |1: Trigger actuator ID assignment and direction mapping.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    PREFLIGHT_STORAGE=245, /* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: start logging with rate of param 3 in Hz (e.g. set to 1000 for 1000 Hz logging)| Reserved| Empty| Empty| Empty|  */
    PREFLIGHT_REBOOT_SHUTDOWN=246, /* Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.| WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded| WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded| Reserved, send 0| Reserved, send 0| WIP: ID (e.g. camera ID -1 for all IDs)|  */
    OVERRIDE_GOTO=252, /* Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  */
    MISSION_START=300, /* start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)|  */
    COMPONENT_ARM_DISARM=400, /* Arms / Disarms a component |1 to arm, 0 to disarm|  */
    GET_HOME_POSITION=410, /* Request the home position from the vehicle. |Reserved| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    START_RX_PAIR=500, /* Starts receiver pairing |0:Spektrum| RC type (see RC_TYPE enum)|  */
    GET_MESSAGE_INTERVAL=510, /* Request the interval between messages for a particular MAVLink message ID |The MAVLink message ID|  */
    SET_MESSAGE_INTERVAL=511, /* Request the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM |The MAVLink message ID| The interval between two messages, in microseconds. Set to -1 to disable and 0 to request default rate.|  */
    REQUEST_PROTOCOL_VERSION=519, /* Request MAVLink protocol version compatibility |1: Request supported protocol versions by all nodes on the network| Reserved (all remaining params)|  */
    REQUEST_AUTOPILOT_CAPABILITIES=520, /* Request autopilot capabilities |1: Request autopilot version| Reserved (all remaining params)|  */
    REQUEST_CAMERA_INFORMATION=521, /* Request camera information (CAMERA_INFORMATION). |0: No action 1: Request camera capabilities| Reserved (all remaining params)|  */
    REQUEST_CAMERA_SETTINGS=522, /* Request camera settings (CAMERA_SETTINGS). |0: No Action 1: Request camera settings| Reserved (all remaining params)|  */
    REQUEST_STORAGE_INFORMATION=525, /* WIP: Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage. |Storage ID (0 for all, 1 for first, 2 for second, etc.)| 0: No Action 1: Request storage information| Reserved (all remaining params)|  */
    STORAGE_FORMAT=526, /* WIP: Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage. |Storage ID (1 for first, 2 for second, etc.)| 0: No action 1: Format storage| Reserved (all remaining params)|  */
    REQUEST_CAMERA_CAPTURE_STATUS=527, /* Request camera capture status (CAMERA_CAPTURE_STATUS) |0: No Action 1: Request camera capture status| Reserved (all remaining params)|  */
    REQUEST_FLIGHT_INFORMATION=528, /* WIP: Request flight information (FLIGHT_INFORMATION) |1: Request flight information| Reserved (all remaining params)|  */
    RESET_CAMERA_SETTINGS=529, /* Reset all camera settings to Factory Default |0: No Action 1: Reset all settings| Reserved (all remaining params)|  */
    SET_CAMERA_MODE=530, /* Set camera running mode. Use NAN for reserved values. |Reserved (Set to 0)| Camera mode (see CAMERA_MODE enum)| Reserved (all remaining params)|  */
    IMAGE_START_CAPTURE=2000, /* Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NAN for reserved values. |Reserved (Set to 0)| Duration between two consecutive pictures (in seconds)| Number of images to capture total - 0 for unlimited capture| Capture sequence (ID to prevent double captures when a command is retransmitted, 0: unused, >= 1: used)| Reserved (all remaining params)|  */
    IMAGE_STOP_CAPTURE=2001, /* Stop image capture sequence Use NAN for reserved values. |Reserved (Set to 0)| Reserved (all remaining params)|  */
    REQUEST_CAMERA_IMAGE_CAPTURE=2002, /* WIP: Re-request a CAMERA_IMAGE_CAPTURE packet. Use NAN for reserved values. |Sequence number for missing CAMERA_IMAGE_CAPTURE packet| Reserved (all remaining params)|  */
    DO_TRIGGER_CONTROL=2003, /* Enable or disable on-board camera triggering system. |Trigger enable/disable (0 for disable, 1 for start), -1 to ignore| 1 to reset the trigger sequence, -1 or 0 to ignore| 1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore|  */
    VIDEO_START_CAPTURE=2500, /* Starts video capture (recording). Use NAN for reserved values. |Reserved (Set to 0)| Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency in Hz)| Reserved (all remaining params)|  */
    VIDEO_STOP_CAPTURE=2501, /* Stop the current video capture (recording). Use NAN for reserved values. |Reserved (Set to 0)| Reserved (all remaining params)|  */
    VIDEO_START_STREAMING=2502, /* WIP: Start video streaming |Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)| Reserved|  */
    VIDEO_STOP_STREAMING=2503, /* WIP: Stop the current video streaming |Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)| Reserved|  */
    REQUEST_VIDEO_STREAM_INFORMATION=2504, /* WIP: Request video stream information (VIDEO_STREAM_INFORMATION) |Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)| 0: No Action 1: Request video stream information| Reserved (all remaining params)|  */
    LOGGING_START=2510, /* Request to start streaming logging data over MAVLink (see also LOGGING_DATA message) |Format: 0: ULog| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
    LOGGING_STOP=2511, /* Request to stop streaming log data over MAVLink |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
    AIRFRAME_CONFIGURATION=2520, /*  |Landing gear ID (default: 0, -1 for all)| Landing gear position (Down: 0, Up: 1, NAN for no change)| Reserved, set to NAN| Reserved, set to NAN| Reserved, set to NAN| Reserved, set to NAN| Reserved, set to NAN|  */
    CONTROL_HIGH_LATENCY=2600, /* Request to start/stop transmitting over the high latency telemetry |Control transmittion over high latency telemetry (0: stop, 1: start)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    PANORAMA_CREATE=2800, /* Create a panorama at the current position |Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)| Viewing angle vertical of panorama (in degrees)| Speed of the horizontal rotation (in degrees per second)| Speed of the vertical rotation (in degrees per second)|  */
    DO_VTOL_TRANSITION=3000, /* Request VTOL transition |The target VTOL state, as defined by ENUM MAV_VTOL_STATE. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.|  */
    ARM_AUTHORIZATION_REQUEST=3001, /* Request authorization to arm the vehicle to a external entity, the arm authorizer is resposible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
         |Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle|  */
    SET_GUIDED_SUBMODE_STANDARD=4000, /* This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocites along all three axes.
                   | */
    SET_GUIDED_SUBMODE_CIRCLE=4001, /* This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
                   |Radius of desired circle in CIRCLE_MODE| User defined| User defined| User defined| Unscaled target latitude of center of circle in CIRCLE_MODE| Unscaled target longitude of center of circle in CIRCLE_MODE|  */
    CONDITION_GATE=4501, /* WIP: Delay mission state machine until gate has been reached. |Geometry: 0: orthogonal to path between previous and next waypoint.| Altitude: 0: ignore altitude| Empty| Empty| Latitude| Longitude| Altitude|  */
    NAV_FENCE_RETURN_POINT=5000, /* Fence return point. There can only be one fence return point.
         |Reserved| Reserved| Reserved| Reserved| Latitude| Longitude| Altitude|  */
    NAV_FENCE_POLYGON_VERTEX_INCLUSION=5001, /* Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
         |Polygon vertex count| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
    NAV_FENCE_POLYGON_VERTEX_EXCLUSION=5002, /* Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
         |Polygon vertex count| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
    NAV_FENCE_CIRCLE_INCLUSION=5003, /* Circular fence area. The vehicle must stay inside this area.
         |radius in meters| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
    NAV_FENCE_CIRCLE_EXCLUSION=5004, /* Circular fence area. The vehicle must stay outside this area.
         |radius in meters| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
    NAV_RALLY_POINT=5100, /* Rally point. You can have multiple rally points defined.
         |Reserved| Reserved| Reserved| Reserved| Latitude| Longitude| Altitude|  */
    UAVCAN_GET_NODE_INFO=5200, /* Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages. |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
    DO_NOTHING=10001, /* Does nothing. |1 to arm, 0 to disarm|  */
    RETURN_TO_BASE=10011, /* Return vehicle to base. |0: return to base, 1: track mobile base|  */
    STOP_RETURN_TO_BASE=10012, /* Stops the vehicle from returning to base and resumes flight.  | */
    TURN_LIGHT=10013, /* Turns the vehicle's visible or infrared lights on or off. |0: visible lights, 1: infrared lights| 0: turn on, 1: turn off|  */
    GET_MID_LEVEL_COMMANDS=10014, /* Requests vehicle to send current mid-level commands to ground station. | */
    MIDLEVEL_STORAGE=10015, /* Requests storage of mid-level commands. |Mid-level command storage: 0: read from flash/EEPROM, 1: write to flash/EEPROM|  */
    PAYLOAD_PREPARE_DEPLOY=30001, /* Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. |Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.| Desired approach vector in degrees compass heading (0..360). A negative value indicates the system can define the approach vector at will.| Desired ground speed at release time. This can be overriden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.| Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.| Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Altitude, in meters AMSL|  */
    PAYLOAD_CONTROL_DEPLOY=30002, /* Control the payload deployment. |Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deploment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    WAYPOINT_USER_1=31000, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    WAYPOINT_USER_2=31001, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    WAYPOINT_USER_3=31002, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    WAYPOINT_USER_4=31003, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    WAYPOINT_USER_5=31004, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    SPATIAL_USER_1=31005, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    SPATIAL_USER_2=31006, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    SPATIAL_USER_3=31007, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    SPATIAL_USER_4=31008, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    SPATIAL_USER_5=31009, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    USER_1=31010, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    USER_2=31011, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    USER_3=31012, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    USER_4=31013, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    USER_5=31014, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
};

//! MAV_CMD ENUM_END
constexpr auto MAV_CMD_ENUM_END = 31015;

/** @brief Slugs-specific navigation modes. */
enum class SLUGS_MODE
{
    NONE=0, /* No change to SLUGS mode. | */
    LIFTOFF=1, /* Vehicle is in liftoff mode. | */
    PASSTHROUGH=2, /* Vehicle is in passthrough mode, being controlled by a pilot. | */
    WAYPOINT=3, /* Vehicle is in waypoint mode, navigating to waypoints. | */
    MID_LEVEL=4, /* Vehicle is executing mid-level commands. | */
    RETURNING=5, /* Vehicle is returning to the home location. | */
    LANDING=6, /* Vehicle is landing. | */
    LOST=7, /* Lost connection with vehicle. | */
    SELECTIVE_PASSTHROUGH=8, /* Vehicle is in selective passthrough mode, where selected surfaces are being manually controlled. | */
    ISR=9, /* Vehicle is in ISR mode, performing reconaissance at a point specified by ISR_LOCATION message. | */
    LINE_PATROL=10, /* Vehicle is patrolling along lines between waypoints. | */
    GROUNDED=11, /* Vehicle is grounded or an error has occurred. | */
};

//! SLUGS_MODE ENUM_END
constexpr auto SLUGS_MODE_ENUM_END = 12;

/** @brief These flags encode the control surfaces for selective passthrough mode. If a bit is set then the pilot console
            has control of the surface, and if not then the autopilot has control of the surface. */
enum class CONTROL_SURFACE_FLAG
{
    RIGHT_FLAP=1, /* 0b00000001 Right flap control passes through to pilot console. | */
    LEFT_FLAP=2, /* 0b00000010 Left flap control passes through to pilot console. | */
    RIGHT_ELEVATOR=4, /* 0b00000100 Right elevator control passes through to pilot console. | */
    LEFT_ELEVATOR=8, /* 0b00001000 Left elevator control passes through to pilot console. | */
    RUDDER=16, /* 0b00010000 Rudder control passes through to pilot console. | */
    RIGHT_AILERON=32, /* 0b00100000 Right aileron control passes through to pilot console. | */
    LEFT_AILERON=64, /* 0b01000000 Left aileron control passes through to pilot console. | */
    THROTTLE=128, /* 0b10000000 Throttle control passes through to pilot console. | */
};

//! CONTROL_SURFACE_FLAG ENUM_END
constexpr auto CONTROL_SURFACE_FLAG_ENUM_END = 129;


} // namespace slugs
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_cpu_load.hpp"
#include "./mavlink_msg_sensor_bias.hpp"
#include "./mavlink_msg_diagnostic.hpp"
#include "./mavlink_msg_slugs_navigation.hpp"
#include "./mavlink_msg_data_log.hpp"
#include "./mavlink_msg_gps_date_time.hpp"
#include "./mavlink_msg_mid_lvl_cmds.hpp"
#include "./mavlink_msg_ctrl_srfc_pt.hpp"
#include "./mavlink_msg_slugs_camera_order.hpp"
#include "./mavlink_msg_control_surface.hpp"
#include "./mavlink_msg_slugs_mobile_location.hpp"
#include "./mavlink_msg_slugs_configuration_camera.hpp"
#include "./mavlink_msg_isr_location.hpp"
#include "./mavlink_msg_volt_sensor.hpp"
#include "./mavlink_msg_ptz_status.hpp"
#include "./mavlink_msg_uav_status.hpp"
#include "./mavlink_msg_status_gps.hpp"
#include "./mavlink_msg_novatel_diag.hpp"
#include "./mavlink_msg_sensor_diag.hpp"
#include "./mavlink_msg_boot.hpp"

// base include
#include "../common/common.hpp"
