# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "C:\\CoffeeWithEmbedded\\STM_Projects\\LedProject\\CM4\\build"
  "C:\\CoffeeWithEmbedded\\STM_Projects\\LedProject\\CM7\\build"
  )
endif()
