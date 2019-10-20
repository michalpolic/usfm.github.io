Thank you for downloading the USfM framework. The code allows to compute the uncertainty of Large scale SfM for COLMAP reconstructions.

Created by: Michal Polic

How to build:
1) Prerequisities 
- CMake   (https://cmake.org)
- Visual Studio on Windows  (https://visualstudio.microsoft.com)
- GCC on Linux 
- VCPKG   (follow instructions on: https://github.com/Microsoft/vcpkg)
- Git     (we recommend to install: https://git-scm.com and https://sourceforge.net/projects/gitextensions/)
- USfM    (you can use GitExtensions - right clic in parent dir and use GitExt Clone ... )

How to build on Windows:
2) Download and build Ceres optimization solver using VCPKG (e.g., run: <path/to/vcpkg.exe> install ceres[suitesparse] --triplet x64-windows)
3) Use Cmake to create the project: 
3.1) Create build dir in the project root dir
3.2) Run cmake-gui in build dir and setup the variables:
- CMAKE_TOOLCHAIN_FILE = %VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake
- VCPKG_TARGET_TRIPLET = x64-windows
(the system variable VCPKG_ROOT have to be setup to the vcpkg root directory, any change of system variables is seen by programs after their restart) 
3.3) [OPTIONAL] It is highly recommended to use some optimized BLAS and LAPACK library otherwise the code will be sllow. Please add:
- MKL_INCLUDE_DIR = <path to MKL include, e.g., C:/Program Files (x86)/IntelSWTools/compilers_and_libraries_2019.4.245/windows/mkl/include>
- MKL_LIBRARIES = <path to MKL libraries, use https://software.intel.com/en-us/articles/intel-mkl-link-line-advisor to select propper version>
3.4) To enable Matlab, check: USfM_BUILD_MEX = ON 
3.5) Run "Configure" (don't forget to select win64 option) and "Generate" 
4) Open the Visual Studio project, right clic on usfm root subroject in Solution Explorer and add in Properties > VC++ Directories > Library directories the path to compiler librearies (e.g. c:/Program Files (x86)/IntelSWTools/compilers_and_libraries_2017.4.210/windows/compiler/lib/intel64_win)
5) The Solution Explorer should contain:
- usfm ... the main library
- uncetainty ... executable application
- [optional] usfm_mex ... the mex file for run from Matlab

How to build on Linux:
1) Instal ceres acording: http://ceres-solver.org
2) Use Cmake to create the makefile in "build" directory, e.g., 
> cd usfm.github.io
> mkdir build
> cd build
> cmake .. 
[OPTIONAL] you can setup the camke variables as in Windows version, see. 4.2, 4.3, 4.4. In command line use -D<name of the variable>=<value of the variable> to setup variables.
3) Build the makefile:
> make 
