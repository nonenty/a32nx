@echo off

:: go to current directory
pushd %~dp0

:: clean build directory
rd /s /q build

:: create build files
cmake -B build

:: build
cmake --build build --config Release

:: copy result
copy build\Release\fdr2csv.exe .

:: restore directory
popd
