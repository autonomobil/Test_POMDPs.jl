using Conda
using RobotOS # ROS stuff for julia


# import ros messages
@rosimport geometry_msgs.msg:Point, Pose2D
@rosimport definitions.msg:IkaEgoState,
IkaObject,
IkaObjectList,
IkaRouteStatus,
IkaRoute,
IkaActionSequence,
IkaObjectListPrediction,
IkaObjectPrediction

rostypegen(@__MODULE__)

import .geometry_msgs.msg: Point, Pose2D
import .definitions.msg:
    IkaEgoState,
    IkaObject,
    IkaObjectList,
    IkaRouteStatus,
    IkaRoute,
    IkaActionSequence,
    IkaObjectListPrediction,
    IkaObjectPrediction
