cmake_minimum_required(VERSION 3.5)
project(fdr2csv LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
        AFTER
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/src/commandline"
        "${CMAKE_SOURCE_DIR}/../fbw/src/model"
)

add_executable(
        fdr2csv
        src/commandline/CommandLine.cpp
        src/FlightDataRecorderConverter.cpp
        src/main.cpp
)
