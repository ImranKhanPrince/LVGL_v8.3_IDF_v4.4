# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Notebook/esp/v4.4/esp-idf/components/bootloader/subproject"
  "C:/Users/Notebook/idf-proj/lvgl_porting/build/bootloader"
  "C:/Users/Notebook/idf-proj/lvgl_porting/build/bootloader-prefix"
  "C:/Users/Notebook/idf-proj/lvgl_porting/build/bootloader-prefix/tmp"
  "C:/Users/Notebook/idf-proj/lvgl_porting/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/Notebook/idf-proj/lvgl_porting/build/bootloader-prefix/src"
  "C:/Users/Notebook/idf-proj/lvgl_porting/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/Notebook/idf-proj/lvgl_porting/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
