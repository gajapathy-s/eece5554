# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "gnss: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ignss:/home/sri/gnss/src/gnss/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(gnss_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sri/gnss/src/gnss/msg/Customgps.msg" NAME_WE)
add_custom_target(_gnss_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gnss" "/home/sri/gnss/src/gnss/msg/Customgps.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(gnss
  "/home/sri/gnss/src/gnss/msg/Customgps.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gnss
)

### Generating Services

### Generating Module File
_generate_module_cpp(gnss
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gnss
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(gnss_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(gnss_generate_messages gnss_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sri/gnss/src/gnss/msg/Customgps.msg" NAME_WE)
add_dependencies(gnss_generate_messages_cpp _gnss_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gnss_gencpp)
add_dependencies(gnss_gencpp gnss_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gnss_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(gnss
  "/home/sri/gnss/src/gnss/msg/Customgps.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gnss
)

### Generating Services

### Generating Module File
_generate_module_eus(gnss
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gnss
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(gnss_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(gnss_generate_messages gnss_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sri/gnss/src/gnss/msg/Customgps.msg" NAME_WE)
add_dependencies(gnss_generate_messages_eus _gnss_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gnss_geneus)
add_dependencies(gnss_geneus gnss_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gnss_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(gnss
  "/home/sri/gnss/src/gnss/msg/Customgps.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gnss
)

### Generating Services

### Generating Module File
_generate_module_lisp(gnss
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gnss
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(gnss_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(gnss_generate_messages gnss_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sri/gnss/src/gnss/msg/Customgps.msg" NAME_WE)
add_dependencies(gnss_generate_messages_lisp _gnss_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gnss_genlisp)
add_dependencies(gnss_genlisp gnss_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gnss_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(gnss
  "/home/sri/gnss/src/gnss/msg/Customgps.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gnss
)

### Generating Services

### Generating Module File
_generate_module_nodejs(gnss
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gnss
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(gnss_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(gnss_generate_messages gnss_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sri/gnss/src/gnss/msg/Customgps.msg" NAME_WE)
add_dependencies(gnss_generate_messages_nodejs _gnss_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gnss_gennodejs)
add_dependencies(gnss_gennodejs gnss_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gnss_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(gnss
  "/home/sri/gnss/src/gnss/msg/Customgps.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gnss
)

### Generating Services

### Generating Module File
_generate_module_py(gnss
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gnss
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(gnss_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(gnss_generate_messages gnss_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sri/gnss/src/gnss/msg/Customgps.msg" NAME_WE)
add_dependencies(gnss_generate_messages_py _gnss_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gnss_genpy)
add_dependencies(gnss_genpy gnss_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gnss_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gnss)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gnss
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(gnss_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gnss)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gnss
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(gnss_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gnss)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gnss
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(gnss_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gnss)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gnss
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(gnss_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gnss)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gnss\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gnss
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(gnss_generate_messages_py std_msgs_generate_messages_py)
endif()
