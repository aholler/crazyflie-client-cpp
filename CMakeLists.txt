#
# Copyright (c) 2014 Alexander Holler <holler@ahsoftware.de>
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

cmake_minimum_required(VERSION 2.6)
project(cfheadless)

SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREAD ON)
find_package(Boost 1.35.0)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

include_directories("${PROJECT_SOURCE_DIR}/libcflie.git/include")

find_library(USB_LIB usb-1.0)

add_executable(${PROJECT_NAME}
	cfheadless.cpp
	libcflie.git/src/cflie/CCrazyRadio.cpp
	libcflie.git/src/cflie/CCrazyflie.cpp
	libcflie.git/src/cflie/CTOC.cpp
)

target_link_libraries(${PROJECT_NAME} ${USB_LIB})

if(CMAKE_COMPILER_IS_GNUCXX)
  # enable c++11 and some other reasonable flags for gcc
  set(MY_COMPILE_FLAGS "-std=gnu++11 -pthread -Wall -Wextra -Wno-strict-aliasing -fvisibility-inlines-hidden")
  set(MY_LINK_FLAGS "-Wl,--as-needed ${MY_COMPILE_FLAGS}")
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES
    COMPILE_FLAGS "${MY_COMPILE_FLAGS}"
    LINK_FLAGS "${MY_LINK_FLAGS}"
    LINK_FLAGS_RELEASE "-s"
  )
endif()

add_custom_target(version.h
    ${CMAKE_CURRENT_SOURCE_DIR}/git-version-gen.sh
        ${CMAKE_CURRENT_BINARY_DIR}/version.h
        ${CMAKE_CURRENT_SOURCE_DIR}/version
        ${CMAKE_CURRENT_SOURCE_DIR} )

ADD_DEPENDENCIES(${PROJECT_NAME} version.h)
SET_SOURCE_FILES_PROPERTIES(${CMAKE_CURRENT_BINARY_DIR}/version.h PROPERTIES GENERATED 1)
