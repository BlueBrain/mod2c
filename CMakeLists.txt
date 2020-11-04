# Copyright (c) 2016, Blue Brain Project
# All rights reserved.

# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.


cmake_minimum_required(VERSION 3.7)
set(CMAKE_BUILD_TYPE Release)

project(mod2c VERSION 2.1.0)

set(MOD2C_DESCRIPTION "MOD2C converter")

set(VERSION_STRING "${PROJECT_VERSION}")

# put mod2c_core in <build>/bin of the project of which
# this is a subproject
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(UNIT_TESTS "Enable unit tests compilation and execution" FALSE)
option(FUNCTIONAL_TESTS "Enable functional tests compilation and execution" FALSE)
option(MOD2C_ENABLE_LEGACY_UNITS "Enable old definition for units" OFF)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake
      ${PROJECT_SOURCE_DIR}/CMake/config)

include(DefineInstallationPaths)
include(ReleaseDebugAutoFlags)

find_package(BISON REQUIRED)
find_package(FLEX REQUIRED)

if(MINGW)
  add_definitions("-DMINGW")
endif()

add_subdirectory(src/mod2c_core)

# Disable tests if we are a sub project
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    include(CTest)
    add_subdirectory(test/validation)
endif()

if (MOD2C_ENABLE_LEGACY_UNITS)
  set(LegacyY "")
  set(LegacyN "/")
else()
  set(LegacyY "/")
  set(LegacyN "")
endif()

configure_file(share/nrnunits.lib.in ${CMAKE_BINARY_DIR}/share/mod2c/nrnunits.lib @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/share/mod2c/nrnunits.lib DESTINATION share/mod2c)
