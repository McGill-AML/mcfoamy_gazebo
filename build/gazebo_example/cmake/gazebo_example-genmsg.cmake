# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "gazebo_example: 1 messages, 0 services")

set(MSG_I_FLAGS "-Igazebo_example:/home/eitan/mcfoamy_gazebo/src/gazebo_example/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(gazebo_example_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/eitan/mcfoamy_gazebo/src/gazebo_example/msg/actuator.msg" NAME_WE)
add_custom_target(_gazebo_example_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gazebo_example" "/home/eitan/mcfoamy_gazebo/src/gazebo_example/msg/actuator.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(gazebo_example
  "/home/eitan/mcfoamy_gazebo/src/gazebo_example/msg/actuator.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gazebo_example
)

### Generating Services

### Generating Module File
_generate_module_cpp(gazebo_example
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gazebo_example
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(gazebo_example_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(gazebo_example_generate_messages gazebo_example_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/eitan/mcfoamy_gazebo/src/gazebo_example/msg/actuator.msg" NAME_WE)
add_dependencies(gazebo_example_generate_messages_cpp _gazebo_example_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gazebo_example_gencpp)
add_dependencies(gazebo_example_gencpp gazebo_example_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gazebo_example_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(gazebo_example
  "/home/eitan/mcfoamy_gazebo/src/gazebo_example/msg/actuator.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gazebo_example
)

### Generating Services

### Generating Module File
_generate_module_eus(gazebo_example
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gazebo_example
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(gazebo_example_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(gazebo_example_generate_messages gazebo_example_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/eitan/mcfoamy_gazebo/src/gazebo_example/msg/actuator.msg" NAME_WE)
add_dependencies(gazebo_example_generate_messages_eus _gazebo_example_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gazebo_example_geneus)
add_dependencies(gazebo_example_geneus gazebo_example_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gazebo_example_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(gazebo_example
  "/home/eitan/mcfoamy_gazebo/src/gazebo_example/msg/actuator.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gazebo_example
)

### Generating Services

### Generating Module File
_generate_module_lisp(gazebo_example
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gazebo_example
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(gazebo_example_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(gazebo_example_generate_messages gazebo_example_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/eitan/mcfoamy_gazebo/src/gazebo_example/msg/actuator.msg" NAME_WE)
add_dependencies(gazebo_example_generate_messages_lisp _gazebo_example_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gazebo_example_genlisp)
add_dependencies(gazebo_example_genlisp gazebo_example_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gazebo_example_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(gazebo_example
  "/home/eitan/mcfoamy_gazebo/src/gazebo_example/msg/actuator.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gazebo_example
)

### Generating Services

### Generating Module File
_generate_module_nodejs(gazebo_example
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gazebo_example
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(gazebo_example_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(gazebo_example_generate_messages gazebo_example_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/eitan/mcfoamy_gazebo/src/gazebo_example/msg/actuator.msg" NAME_WE)
add_dependencies(gazebo_example_generate_messages_nodejs _gazebo_example_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gazebo_example_gennodejs)
add_dependencies(gazebo_example_gennodejs gazebo_example_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gazebo_example_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(gazebo_example
  "/home/eitan/mcfoamy_gazebo/src/gazebo_example/msg/actuator.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gazebo_example
)

### Generating Services

### Generating Module File
_generate_module_py(gazebo_example
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gazebo_example
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(gazebo_example_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(gazebo_example_generate_messages gazebo_example_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/eitan/mcfoamy_gazebo/src/gazebo_example/msg/actuator.msg" NAME_WE)
add_dependencies(gazebo_example_generate_messages_py _gazebo_example_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gazebo_example_genpy)
add_dependencies(gazebo_example_genpy gazebo_example_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gazebo_example_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gazebo_example)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gazebo_example
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(gazebo_example_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gazebo_example)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gazebo_example
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(gazebo_example_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gazebo_example)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gazebo_example
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(gazebo_example_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gazebo_example)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gazebo_example
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(gazebo_example_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gazebo_example)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gazebo_example\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gazebo_example
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gazebo_example
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gazebo_example/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(gazebo_example_generate_messages_py std_msgs_generate_messages_py)
endif()
