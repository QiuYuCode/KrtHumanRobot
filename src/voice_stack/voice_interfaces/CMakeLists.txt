cmake_minimum_required(VERSION 3.8)
project(voice_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VoiceAudioFrame.msg"
  "msg/VoiceKwsEvent.msg"
  "msg/VoiceVadEvent.msg"
  "srv/RecognizeSpeech.srv"
  "srv/SynthesizeSpeech.srv"
  "srv/SetVolume.srv"
  "srv/GetVolume.srv"
  "srv/SetMute.srv"
  "srv/StopPlayback.srv"
  "action/RecognizeStream.action"
  "action/PlayAudio.action"
  DEPENDENCIES builtin_interfaces std_msgs unique_identifier_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
