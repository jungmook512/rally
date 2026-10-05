cmake_minimum_required(VERSION 3.8)
project(rally_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  "msg/HandState.msg"
  "msg/BodyPose.msg"
  "msg/VoiceIntent.msg"
  "msg/SafetyStatus.msg"
  "msg/TrialResult.msg"
  "msg/SceneContext.msg"
  "srv/SetMode.srv"
  "srv/RunCalibration.srv"
  "srv/SuggestParams.srv"
  "srv/Speak.srv"
  "srv/GripperCommand.srv"
  "srv/SrvDepthPosition.srv"
  "action/PushBall.action"
  "action/MoveTarget.action"
  "action/DetachBall.action"
  "action/Handover.action"
  "action/PickCompliant.action"
  "action/RunSession.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
