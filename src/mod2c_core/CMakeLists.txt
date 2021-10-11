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


set(NOCMODL_CORE_SOURCES
"consist.c"
"io.c"       "list.c"
"nocpout.c"  "partial.c"
"solve.c"    "version.c"
"discrete.c"
"kinetic.c"  "modl.c"
"parsact.c"  "sens.c"
"symbol.c"   "deriv.c"
"init.c"
"noccout.c"
"simultan.c" "units.c"
)

bison_target(diffeq diffeq.y ${CMAKE_CURRENT_BINARY_DIR}/diffeq.c)
bison_target(parse1 parse1.y ${CMAKE_CURRENT_BINARY_DIR}/parse1.c)

flex_target(lex lex.l ${CMAKE_CURRENT_BINARY_DIR}/lex.c)
add_flex_bison_dependency(lex parse1)

if(MOD2C_ENABLE_LEGACY_UNITS)
    set(USE_LEGACY_UNITS 1)
else()
    set(USE_LEGACY_UNITS 0)
endif()

add_definitions(-DNMODL=1 -DBBCORE=1 -DNOCMODL=1 -DCVODE=1 -DVECTORIZE=1
    -DUSE_LEGACY_UNITS=${USE_LEGACY_UNITS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

add_executable(mod2c_core
    ${NOCMODL_CORE_SOURCES}
    ${BISON_diffeq_OUTPUTS} ${BISON_parse1_OUTPUTS}
    ${FLEX_lex_OUTPUTS})

# as mod2c is typically executed on front-end node, in order to avoid runtime
# issues from platform specific optimisations, build mod2c with debug flags
# as performance is not a concern (for translating mod files to cpp)
separate_arguments(NMODL_C_FLAGS UNIX_COMMAND "${CMAKE_CXX_FLAGS_DEBUG}")

if(CMAKE_C_COMPILER_ID STREQUAL "PGI" OR CMAKE_C_COMPILER_ID STREQUAL "NVHPC")
  if(CMAKE_C_COMPILER_VERSION VERSION_LESS 20.7)
    # Taken from CoreNEURON's OpenAccHelper.cmake
    list(APPEND NMODL_C_FLAGS --diag_suppress=161,177,550)
  else()
    # https://forums.developer.nvidia.com/t/many-all-diagnostic-numbers-increased-by-1-from-previous-values/146268/3
    # changed the numbering scheme in newer versions. The following list is from a clean start 13
    # August 2021.
    # Examples of the suppressed warnings are given below.
    # ~~~
    # "src/mod2c_core/kinetic.c", warning #111-D: statement is unreachable
    # "src/mod2c_core/parsact.c", warning #128-D: loop is not reachable
    # "src/mod2c_core/units.c", warning #170-D: pointer points outside of underlying object
    # "src/mod2c_core/parsact.c", warning #177-D: variable "qq" was declared but never referenced
    # "src/mod2c_core/kinetic.c", warning #550-D: variable "qv" was set but never used
    # "src/mod2c_core/sens.c", warning #1052-D: standard requires that parameter "fn" be given a type by a subsequent declaration ("int" assumed)
    # ~~~
    list(APPEND NMODL_C_FLAGS --diag_suppress=111,128,170,177,550,1052)
  endif()
endif()
target_compile_options(mod2c_core PRIVATE ${NMODL_C_FLAGS})

install(TARGETS mod2c_core DESTINATION bin)

