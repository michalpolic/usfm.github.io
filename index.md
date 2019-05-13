## Welcome to Uncertainty4SfM Pages

This repository contains the original source codes used for ECCV2018 publication: [Fast and Accurate Camera Covariance Computation for Large 3D Reconstruction](http://people.ciirc.cvut.cz/~policmic/files/Polic_Fast_and_Accurate_Camera_Covariance.pdf)

### External libraries
Requirements: 
 - Gflags, Eigen, Blas and Lapack, Ceres, Cuda
Optional:
 - Glog, Matlab, SuiteSparse

### Installation 
How to build on Windows:
1) Open command line: "VS2015 x64 Native Tools Command Prompt"
2) Download and install Vcpkg package manager for Windows (follow instructions on: https://github.com/Microsoft/vcpkg)
3) Install Ceres nonlinear optimization solver (run: <path/to/vcpkg.exe> install ceres[suitesparse])
4) Setup the system veriable VCPKG_ROOT to the vcpkg root directory and restart command prompt (step 0)
5) Use Cmake to create the project:
cmake .. -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows -G "Visual Studio 14 2015" -A x64 -T v140,host=x64
6) Compile the project

How to build on Linux:
1) Instal ceres acording: http://ceres-solver.org
2) Use Cmake to create the makefile and compile it

### Output
The codes can be used to save the covarince matrix as a txt file, called from the Matlab or called as an external library. The covariance matrices are saved as submatrices related to camera blocks folowed by submatrices related to points in 3D blocks. See the code: https://github.com/michalpolic/usfm.github.io/blob/master/src/USfM/usfm_IO.cpp.

### Documantation and AP
The executable with "--help" argument prints how to use it.

### Support or Contact

Detailed information may be on my [web page](http://people.ciirc.cvut.cz/~policmic).
If you have any questions please contact me and I’ll help you sort it out.
