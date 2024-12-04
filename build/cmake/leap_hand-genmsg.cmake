# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "leap_hand: 0 messages, 3 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(leap_hand_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_velocity.srv" NAME_WE)
add_custom_target(_leap_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "leap_hand" "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_velocity.srv" ""
)

get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_position.srv" NAME_WE)
add_custom_target(_leap_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "leap_hand" "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_position.srv" ""
)

get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_effort.srv" NAME_WE)
add_custom_target(_leap_hand_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "leap_hand" "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_effort.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(leap_hand
  "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_velocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/leap_hand
)
_generate_srv_cpp(leap_hand
  "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_position.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/leap_hand
)
_generate_srv_cpp(leap_hand
  "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_effort.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/leap_hand
)

### Generating Module File
_generate_module_cpp(leap_hand
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/leap_hand
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(leap_hand_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(leap_hand_generate_messages leap_hand_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_velocity.srv" NAME_WE)
add_dependencies(leap_hand_generate_messages_cpp _leap_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_position.srv" NAME_WE)
add_dependencies(leap_hand_generate_messages_cpp _leap_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_effort.srv" NAME_WE)
add_dependencies(leap_hand_generate_messages_cpp _leap_hand_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(leap_hand_gencpp)
add_dependencies(leap_hand_gencpp leap_hand_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS leap_hand_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(leap_hand
  "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_velocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/leap_hand
)
_generate_srv_eus(leap_hand
  "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_position.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/leap_hand
)
_generate_srv_eus(leap_hand
  "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_effort.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/leap_hand
)

### Generating Module File
_generate_module_eus(leap_hand
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/leap_hand
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(leap_hand_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(leap_hand_generate_messages leap_hand_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_velocity.srv" NAME_WE)
add_dependencies(leap_hand_generate_messages_eus _leap_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_position.srv" NAME_WE)
add_dependencies(leap_hand_generate_messages_eus _leap_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_effort.srv" NAME_WE)
add_dependencies(leap_hand_generate_messages_eus _leap_hand_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(leap_hand_geneus)
add_dependencies(leap_hand_geneus leap_hand_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS leap_hand_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(leap_hand
  "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_velocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/leap_hand
)
_generate_srv_lisp(leap_hand
  "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_position.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/leap_hand
)
_generate_srv_lisp(leap_hand
  "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_effort.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/leap_hand
)

### Generating Module File
_generate_module_lisp(leap_hand
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/leap_hand
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(leap_hand_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(leap_hand_generate_messages leap_hand_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_velocity.srv" NAME_WE)
add_dependencies(leap_hand_generate_messages_lisp _leap_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_position.srv" NAME_WE)
add_dependencies(leap_hand_generate_messages_lisp _leap_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_effort.srv" NAME_WE)
add_dependencies(leap_hand_generate_messages_lisp _leap_hand_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(leap_hand_genlisp)
add_dependencies(leap_hand_genlisp leap_hand_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS leap_hand_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(leap_hand
  "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_velocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/leap_hand
)
_generate_srv_nodejs(leap_hand
  "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_position.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/leap_hand
)
_generate_srv_nodejs(leap_hand
  "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_effort.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/leap_hand
)

### Generating Module File
_generate_module_nodejs(leap_hand
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/leap_hand
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(leap_hand_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(leap_hand_generate_messages leap_hand_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_velocity.srv" NAME_WE)
add_dependencies(leap_hand_generate_messages_nodejs _leap_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_position.srv" NAME_WE)
add_dependencies(leap_hand_generate_messages_nodejs _leap_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_effort.srv" NAME_WE)
add_dependencies(leap_hand_generate_messages_nodejs _leap_hand_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(leap_hand_gennodejs)
add_dependencies(leap_hand_gennodejs leap_hand_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS leap_hand_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(leap_hand
  "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_velocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leap_hand
)
_generate_srv_py(leap_hand
  "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_position.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leap_hand
)
_generate_srv_py(leap_hand
  "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_effort.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leap_hand
)

### Generating Module File
_generate_module_py(leap_hand
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leap_hand
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(leap_hand_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(leap_hand_generate_messages leap_hand_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_velocity.srv" NAME_WE)
add_dependencies(leap_hand_generate_messages_py _leap_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_position.srv" NAME_WE)
add_dependencies(leap_hand_generate_messages_py _leap_hand_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/boris/workspace/human-retargeting/3rdparty/LEAP_Hand_API/ros_module/srv/leap_effort.srv" NAME_WE)
add_dependencies(leap_hand_generate_messages_py _leap_hand_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(leap_hand_genpy)
add_dependencies(leap_hand_genpy leap_hand_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS leap_hand_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/leap_hand)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/leap_hand
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(leap_hand_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/leap_hand)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/leap_hand
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(leap_hand_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/leap_hand)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/leap_hand
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(leap_hand_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/leap_hand)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/leap_hand
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(leap_hand_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leap_hand)
  install(CODE "execute_process(COMMAND \"/home/boris/anaconda3/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leap_hand\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/leap_hand
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(leap_hand_generate_messages_py std_msgs_generate_messages_py)
endif()
