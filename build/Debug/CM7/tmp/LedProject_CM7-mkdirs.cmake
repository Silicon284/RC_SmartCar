# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/CoffeeWithEmbedded/STM_Projects/LedProject/CM7"
  "C:/CoffeeWithEmbedded/STM_Projects/LedProject/CM7/build"
  "C:/CoffeeWithEmbedded/STM_Projects/LedProject/build/Debug/CM7"
  "C:/CoffeeWithEmbedded/STM_Projects/LedProject/build/Debug/CM7/tmp"
  "C:/CoffeeWithEmbedded/STM_Projects/LedProject/build/Debug/CM7/src/LedProject_CM7-stamp"
  "C:/CoffeeWithEmbedded/STM_Projects/LedProject/build/Debug/CM7/src"
  "C:/CoffeeWithEmbedded/STM_Projects/LedProject/build/Debug/CM7/src/LedProject_CM7-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/CoffeeWithEmbedded/STM_Projects/LedProject/build/Debug/CM7/src/LedProject_CM7-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/CoffeeWithEmbedded/STM_Projects/LedProject/build/Debug/CM7/src/LedProject_CM7-stamp${cfgdir}") # cfgdir has leading slash
endif()
