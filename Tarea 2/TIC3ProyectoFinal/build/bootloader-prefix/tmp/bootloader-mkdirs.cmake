# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/tomas/esp/v5.3/esp-idf/components/bootloader/subproject"
  "C:/Users/tomas/Tarea2Tic3/TIC3ProyectoFinal/build/bootloader"
  "C:/Users/tomas/Tarea2Tic3/TIC3ProyectoFinal/build/bootloader-prefix"
  "C:/Users/tomas/Tarea2Tic3/TIC3ProyectoFinal/build/bootloader-prefix/tmp"
  "C:/Users/tomas/Tarea2Tic3/TIC3ProyectoFinal/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/tomas/Tarea2Tic3/TIC3ProyectoFinal/build/bootloader-prefix/src"
  "C:/Users/tomas/Tarea2Tic3/TIC3ProyectoFinal/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/tomas/Tarea2Tic3/TIC3ProyectoFinal/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/tomas/Tarea2Tic3/TIC3ProyectoFinal/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
