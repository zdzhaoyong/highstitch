cmake_minimum_required(VERSION 2.8)
project(highstitch)

include(cmake/PICMake.cmake)
set(CMAKE_CXX_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

pi_add_target(highstitch BIN src REQUIRED System)

pi_report_target()

# Now do make install
pi_install(TARGETS highstitch)
