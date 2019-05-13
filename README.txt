Thank you for downloading the USfM framework. 

Created by: Michal Polic

How to build on Windows:
0) Open command line: "VS2015 x64 Native Tools Command Prompt"
1) Download and install Vcpkg package manager for Windows (follow instructions on: https://github.com/Microsoft/vcpkg)
2) Install Ceres nonlinear optimization solver (run: <path/to/vcpkg.exe> install ceres[suitesparse])
3) Setup the system veriable VCPKG_ROOT to the vcpkg root directory and restart command prompt (step 0)
4) Use Cmake to create the project:
cmake .. -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows -G "Visual Studio 14 2015" -A x64 -T v140,host=x64
5) Compile the project

How to build on Linux:
1) Instal ceres acording: http://ceres-solver.org
2) Use Cmake to create the makefile and compile it