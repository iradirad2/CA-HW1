cmake_minimum_required(VERSION 4.00) # Specify the minimum required CMake version

project(BranchPredictor LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(src)

