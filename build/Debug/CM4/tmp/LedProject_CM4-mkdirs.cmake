# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/CoffeeWithEmbedded/STM_Projects/LedProject/CM4"
  "C:/CoffeeWithEmbedded/STM_Projects/LedProject/CM4/build"
  "C:/CoffeeWithEmbedded/STM_Projects/LedProject/build/Debug/CM4"
  "C:/CoffeeWithEmbedded/STM_Projects/LedProject/build/Debug/CM4/tmp"
  "C:/CoffeeWithEmbedded/STM_Projects/LedProject/build/Debug/CM4/src/LedProject_CM4-stamp"
  "C:/CoffeeWithEmbedded/STM_Projects/LedProject/build/Debug/CM4/src"
  "C:/CoffeeWithEmbedded/STM_Projects/LedProject/build/Debug/CM4/src/LedProject_CM4-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/CoffeeWithEmbedded/STM_Projects/LedProject/build/Debug/CM4/src/LedProject_CM4-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/CoffeeWithEmbedded/STM_Projects/LedProject/build/Debug/CM4/src/LedProject_CM4-stamp${cfgdir}") # cfgdir has leading slash
endif()
