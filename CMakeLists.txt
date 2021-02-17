#  Software License Agreement (BSD License)
#  Copyright (c) 2003-2016, CHAI3D.
#  (www.chai3d.org)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of CHAI3D nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: seb $
#  $Date: 2016-02-01 00:37:30 +0100 (Mon, 01 Feb 2016) $
#  $Rev: 1939 $


#
# project configuration
#

cmake_minimum_required (VERSION 3.0.0)
project (CHAI3D-V-REP)

# set version info
file (READ ${PROJECT_SOURCE_DIR}/src/version VERSION)
STRING (REGEX REPLACE ".*MAJOR=([0-9]+).*" "\\1" MAJOR_VERSION "${VERSION}")
STRING (REGEX REPLACE ".*MINOR=([0-9]+).*" "\\1" MINOR_VERSION "${VERSION}")
STRING (REGEX REPLACE ".*RELEASE=([0-9]+).*" "\\1" RELEASE_VERSION "${VERSION}")
set (PROJECT_VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${RELEASE_VERSION})

# platform detection
if (CMAKE_SYSTEM_NAME MATCHES Windows)
  set (OS "win")
  if (CMAKE_CL_64)
    set (ARCH "x64")
  else ()
    set (ARCH "Win32")
  endif ()
elseif (CMAKE_SYSTEM_NAME MATCHES Darwin)
  set (OS "mac")
  if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    set (ARCH "x86_64")
  else ()
    set (ARCH "i386")
  endif ()
elseif (CMAKE_SYSTEM_NAME MATCHES Linux) 
  set (OS "lin")
  if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    set (ARCH "x86_64")
  else ()
    set (ARCH "i686")
  endif ()
endif ()

# output location
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY                ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG          ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL     ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE        ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})

# find dependencies
find_package (CHAI3D REQUIRED)
include_directories (${CHAI3D_INCLUDE_DIRS})
link_directories (${CHAI3D_LIBRARY_DIRS})
add_definitions (${CHAI3D_DEFINITIONS})

# optional settings
include (${PROJECT_SOURCE_DIR}/CMakeOptions.txt OPTIONAL)

# optional configuration (configured based on CMakeOptions.txt)
# -

# Windows global build options
if (CMAKE_SYSTEM_NAME MATCHES Windows)

  # VisualStudio compiler
  if (MSVC)
    add_definitions (-D_CRT_SECURE_NO_DEPRECATE)
    if (CMAKE_CL_64)
      add_definitions (-DWIN64)
    else ()
      add_definitions (-DWIN32)
    endif ()
    
  # MinGW compiler
  elseif (MINGW)
    add_definitions (-DWIN32)
    set (CMAKE_CXX_FLAGS                "${CMAKE_CXX_FLAGS}                -march=native -Wno-deprecated -std=c++0x")
    set (CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG}          -march=native -Wno-deprecated -std=c++0x")
    set (CMAKE_CXX_FLAGS_MINSIZEREL     "${CMAKE_CXX_FLAGS_MINSIZEREL}     -march=native -Wno-deprecated -std=c++0x")
    set (CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE}        -march=native -Wno-deprecated -std=c++0x")
    set (CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -march=native -Wno-deprecated -std=c++0x")
    set (CMAKE_C_FLAGS                  "${CMAKE_C_FLAGS}                  -march=native -Wno-deprecated")
    set (CMAKE_C_FLAGS_DEBUG            "${CMAKE_C_FLAGS_DEBUG}            -march=native -Wno-deprecated")
    set (CMAKE_C_FLAGS_MINSIZEREL       "${CMAKE_C_FLAGS_MINSIZEREL}       -march=native -Wno-deprecated")
    set (CMAKE_C_FLAGS_RELEASE          "${CMAKE_C_FLAGS_RELEASE}          -march=native -Wno-deprecated")
    set (CMAKE_C_FLAGS_RELWITHDEBINFO   "${CMAKE_C_FLAGS_RELWITHDEBINFO}   -march=native -Wno-deprecated")
  endif ()

# Linux global build options
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  add_definitions (-DLINUX)
  set (CMAKE_CXX_FLAGS                "${CMAKE_CXX_FLAGS}                -fPIC -march=native -Wno-deprecated -std=c++0x")
  set (CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG}          -fPIC -march=native -Wno-deprecated -std=c++0x")
  set (CMAKE_CXX_FLAGS_MINSIZEREL     "${CMAKE_CXX_FLAGS_MINSIZEREL}     -fPIC -march=native -Wno-deprecated -std=c++0x")
  set (CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE}        -fPIC -march=native -Wno-deprecated -std=c++0x")
  set (CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -fPIC -march=native -Wno-deprecated -std=c++0x")
  set (CMAKE_C_FLAGS                  "${CMAKE_C_FLAGS}                  -fPIC -march=native -Wno-deprecated")
  set (CMAKE_C_FLAGS_DEBUG            "${CMAKE_C_FLAGS_DEBUG}            -fPIC -march=native -Wno-deprecated")
  set (CMAKE_C_FLAGS_MINSIZEREL       "${CMAKE_C_FLAGS_MINSIZEREL}       -fPIC -march=native -Wno-deprecated")
  set (CMAKE_C_FLAGS_RELEASE          "${CMAKE_C_FLAGS_RELEASE}          -fPIC -march=native -Wno-deprecated")
  set (CMAKE_C_FLAGS_RELWITHDEBINFO   "${CMAKE_C_FLAGS_RELWITHDEBINFO}   -fPIC -march=native -Wno-deprecated")

# Mac OS X global build options
elseif (CMAKE_SYSTEM_NAME MATCHES Darwin)
  add_definitions (-DMACOSX)
  set (CMAKE_CXX_FLAGS                "${CMAKE_CXX_FLAGS}                -Qunused-arguments -Wno-deprecated -std=c++0x -stdlib=libc++")
  set (CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG}          -Qunused-arguments -Wno-deprecated -std=c++0x -stdlib=libc++")
  set (CMAKE_CXX_FLAGS_MINSIZEREL     "${CMAKE_CXX_FLAGS_MINSIZEREL}     -Qunused-arguments -Wno-deprecated -std=c++0x -stdlib=libc++")
  set (CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE}        -Qunused-arguments -Wno-deprecated -std=c++0x -stdlib=libc++")
  set (CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -Qunused-arguments -Wno-deprecated -std=c++0x -stdlib=libc++")
  set (CMAKE_C_FLAGS                  "${CMAKE_C_FLAGS}                  -Qunused-arguments -Wno-deprecated")
  set (CMAKE_C_FLAGS_DEBUG            "${CMAKE_C_FLAGS_DEBUG}            -Qunused-arguments -Wno-deprecated")
  set (CMAKE_C_FLAGS_MINSIZEREL       "${CMAKE_C_FLAGS_MINSIZEREL}       -Qunused-arguments -Wno-deprecated")
  set (CMAKE_C_FLAGS_RELEASE          "${CMAKE_C_FLAGS_RELEASE}          -Qunused-arguments -Wno-deprecated")
  set (CMAKE_C_FLAGS_RELWITHDEBINFO   "${CMAKE_C_FLAGS_RELWITHDEBINFO}   -Qunused-arguments -Wno-deprecated")
endif ()


# 
# shared library
# 

# header search path
include_directories (${PROJECT_SOURCE_DIR}/src)
include_directories (${PROJECT_SOURCE_DIR}/external/V-REP/include)

# shared library source files
file (GLOB_RECURSE source ${PROJECT_SOURCE_DIR}/src/*.cpp)

# shared library external dependencies source files
file (GLOB_RECURSE source_vrep ${PROJECT_SOURCE_DIR}/external/V-REP/common/*.cpp)

# platform-specific source files adjustments
# -

# build flags
add_definitions (${PROJECT_DEFINITIONS})

# shared library
add_library (v_repExtCHAI3D SHARED ${source} ${source_vrep})
target_link_libraries (v_repExtCHAI3D ${CHAI3D_LIBRARIES})
if (CMAKE_SYSTEM_NAME MATCHES Windows)
  target_link_libraries (v_repExtCHAI3D shlwapi)
endif ()
