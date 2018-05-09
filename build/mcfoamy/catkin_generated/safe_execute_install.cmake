execute_process(COMMAND "/home/eitan/mcfoamy_gazebo/build/mcfoamy/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/eitan/mcfoamy_gazebo/build/mcfoamy/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
