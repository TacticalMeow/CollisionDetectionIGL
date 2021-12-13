# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/repos/AnimationAssign2/cmake/../external/libigl-imgui"
  "E:/Repositories/Animation_igl_engine_build_assignment2/bin/libigl-imgui-build"
  "D:/repos/AnimationAssign2/external/.cache/libigl-imgui/libigl-imgui-download-prefix"
  "D:/repos/AnimationAssign2/external/.cache/libigl-imgui/libigl-imgui-download-prefix/tmp"
  "D:/repos/AnimationAssign2/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp"
  "D:/repos/AnimationAssign2/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src"
  "D:/repos/AnimationAssign2/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp"
)

set(configSubDirs Debug;Release;MinSizeRel;RelWithDebInfo)
foreach(subDir IN LISTS configSubDirs)
  file(MAKE_DIRECTORY "D:/repos/AnimationAssign2/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp/${subDir}")
endforeach()
