# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/workspace/SKR-Pico_dev/winder_project/build/_deps/picotool-src/enc_bootloader")
  file(MAKE_DIRECTORY "/workspace/SKR-Pico_dev/winder_project/build/_deps/picotool-src/enc_bootloader")
endif()
file(MAKE_DIRECTORY
  "/workspace/SKR-Pico_dev/winder_project/build/_deps/picotool-build/enc_bootloader"
  "/workspace/SKR-Pico_dev/winder_project/build/_deps/picotool-build/enc_bootloader"
  "/workspace/SKR-Pico_dev/winder_project/build/_deps/picotool-build/enc_bootloader/tmp"
  "/workspace/SKR-Pico_dev/winder_project/build/_deps/picotool-build/enc_bootloader/src/enc_bootloader-stamp"
  "/workspace/SKR-Pico_dev/winder_project/build/_deps/picotool-build/enc_bootloader/src"
  "/workspace/SKR-Pico_dev/winder_project/build/_deps/picotool-build/enc_bootloader/src/enc_bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/workspace/SKR-Pico_dev/winder_project/build/_deps/picotool-build/enc_bootloader/src/enc_bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/workspace/SKR-Pico_dev/winder_project/build/_deps/picotool-build/enc_bootloader/src/enc_bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
