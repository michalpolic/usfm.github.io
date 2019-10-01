CMAKE_MINIMUM_REQUIRED (VERSION 3.13)
PROJECT (USfM LANGUAGES C CXX)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed.")
endif()

# Add path to find_package modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/src/cmake")

option(USfM_BUILD_MEX "Build MEX (require Matlab)" OFF)
option(USfM_BUILD_TESTS "Build unit tests" OFF)
option(USfM_MKL_SUPPORT "Build with MKL" OFF)

# -----------------------------------------------------------------------------
# SYSTEM LIBRARY - FOR LINUX
# -----------------------------------------------------------------------------
if(UNIX)
  find_package (Threads)
  set(LIBS_FILES ${LIBS_FILES} ${CMAKE_THREAD_LIBS_INIT})
	
  find_package(OpenMP)
  if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  endif(OPENMP_FOUND)
endif(UNIX)

# -----------------------------------------------------------------------------
# Gflags
# -----------------------------------------------------------------------------
find_package(gflags REQUIRED)
if(DEFINED gflags_VERSION_STRING OR DEFINED GFLAGS_VERSION_STRING)
        # different cmake variable with lower or upper case exist
        set(GFLAGS_INCLUDE_DIR ${GFLAGS_INCLUDE_DIR} ${gflags_INCLUDE_DIR})
        set(GFLAGS_LIBRARIES ${GFLAGS_LIBRARIES} ${gflags_LIBRARIES})
  message( "Found Gflags ${GFLAGS_VERSION_STRING}" )
  message( STATUS "> GFLAGS_INCLUDE_DIR:   ${GFLAGS_INCLUDE_DIR}" )
  message( STATUS "> GFLAGS_LIBRARIES:   ${GFLAGS_LIBRARIES}" )
endif()


# -----------------------------------------------------------------------------
# EIGEN
# -----------------------------------------------------------------------------
find_package(Eigen3 3.3 NO_MODULE REQUIRED)
if(Eigen3_FOUND)
  message( "Found Eigen${EIGEN3_VERSION_STRING}" )
  message( STATUS "> EIGEN3_INCLUDE_DIRS:   ${EIGEN3_INCLUDE_DIRS}"   )
  set(EIGEN_FLAGS "-DEIGEN_DONT_ALIGN_STATICALLY=1 -DEIGEN_DONT_VECTORIZE=1")
endif()


## -----------------------------------------------------------------------------
## IntelMKL -> BLAS & LAPACK
## -----------------------------------------------------------------------------
if(USfM_MKL_SUPPORT)
	find_package(MKL QUIET)
  if(MKL_FOUND)
	add_definitions(-DUSE_MKL)
    message( "Found Intel MKL" )
    message( STATUS "> MKL_INCLUDE_DIR: ${MKL_INCLUDE_DIR}"   )
    message( STATUS "> MKL_LIBRARIES: ${MKL_LIBRARIES}" )
  else()
	message( WARNING "> Not foud Intel MKL, please setup paths: MKL_INCLUDE_DIR, MKL_LIBRARIES" )
    message( STATUS "> MKL_INCLUDE_DIR: ${MKL_INCLUDE_DIR}"   )
    message( STATUS "> MKL_LIBRARIES: ${MKL_LIBRARIES}" )
  endif()
endif()


# -----------------------------------------------------------------------------
# BLAS and LAPACK
# -----------------------------------------------------------------------------
#find_package(LAPACK REQUIRED)
#if(LAPACK_FOUND)
#  add_definitions(-DUSE_LAPACK)
#  message( "Found LAPACK" )
#  message( STATUS "> LAPACK_INCLUDE_DIRS: 0.3.6  ${LAPACK_INCLUDE_DIRS}" )
#  message( STATUS "> LAPACK_LIBRARIES:   ${LAPACK_LIBRARIES}" )
#endif()


## -----------------------------------------------------------------------------
## LAPACK AND BLAS LIBS
## -----------------------------------------------------------------------------
if(NOT UNC_MKL_SUPPORT AND UNIX)
  find_package(LAPACK REQUIRED)
  if(LAPACK_FOUND)
	#add_definitions(-DUSE_MKL)
    message("Found lapack and blas: ${LAPACK_LIBRARIES}")
  endif()
endif()


# -----------------------------------------------------------------------------
# SuiteSparse 
# -----------------------------------------------------------------------------
#find_package(SuiteSparse REQUIRED)
#if(SuiteSparse_FOUND)
#  message( "Found SuiteSparse" )
#  message( STATUS "> SUITESPARSE_INCLUDE_DIRS:   ${SUITESPARSE_INCLUDE_DIRS} ${SuiteSparse_INCLUDE_DIRS}" )
#  message( STATUS "> SUITESPARSE_LIBRARIES:   ${SUITESPARSE_LIBRARIES} ${SuiteSparse_LIBRARIES}" )
#endif()

# -----------------------------------------------------------------------------
# CERES
# -----------------------------------------------------------------------------
find_package(Ceres REQUIRED)
if(CERES_FOUND)
  message( "Found CERES ${CERES_VERSION}" )
  message( STATUS "> CERES_INCLUDE_DIRS:   ${CERES_INCLUDE_DIRS}"   )
  message( STATUS "> CERES_LIBRARIES: ${CERES_LIBRARIES}" )
endif()

# -----------------------------------------------------------------------------
# MATLAB interface - MEX libs ( C:/Program Files/MATLAB/R2017a/extern/include; C:/Program Files/MATLAB/R2017a/extern/lib/win64/microsoft )
# -----------------------------------------------------------------------------
if(USfM_BUILD_MEX)
  find_package(Matlab)
  if(Matlab_FOUND)
    add_definitions(-DUSE_MATLAB)
    message("Found Matlab libraries")
    message(STATUS "Matlab_INCLUDE_DIRS: ${Matlab_INCLUDE_DIRS}")
    set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Matlab_INCLUDE_DIRS})
	LIST(GET Matlab_LIBRARIES 1 Matlab_LIB)
	get_filename_component(MATLAB_LIBS_ROOT ${Matlab_LIB} DIRECTORY)  # ${Matlab_LIBRARIES} doesn't contain all required libs
    if(UNIX)
      set(Matlab_LIBRARIES ${Matlab_LIBRARIES}
				"${MATLAB_LIBS_ROOT}/libmx.so"
				"${MATLAB_LIBS_ROOT}/libmat.so"
				"${MATLAB_LIBS_ROOT}/libeng.so"
				"${MATLAB_LIBS_ROOT}/libmex.so")
    else()
      set(Matlab_LIBRARIES ${Matlab_LIBRARIES}
				"${MATLAB_LIBS_ROOT}/libmx.lib"
				"${MATLAB_LIBS_ROOT}/libmat.lib"
				"${MATLAB_LIBS_ROOT}/libeng.lib"
				"${MATLAB_LIBS_ROOT}/libmex.lib")
    endif()
	message(STATUS "Matlab_LIBRARIES: ${Matlab_LIBRARIES}")
  endif()
endif()


# -----------------------------------------------------------------------------
# Boost
# -----------------------------------------------------------------------------
#find_package(Boost REQUIRED)
#
#if(Boost_FOUND)
#  message("Found Boost")
#  message(STATUS "> Boost_INCLUDE_DIRS:   ${Boost_INCLUDE_DIRS}")
#  message(STATUS "> Boost_LIBRARIES:   ${Boost_LIBRARIES}")
#else()
#  message(WARNING "Failed to find the Boost library.")
#endif()

# -----------------------------------------------------------------------------
# Add project source code
# -----------------------------------------------------------------------------
add_subdirectory(src)


# -----------------------------------------------------------------------------
# PLATFORM SPECIFIC ERRORS
# The debug on windows may need additional libraries
# -----------------------------------------------------------------------------
if(WIN32)
  set(INTEL_COMPILER_DEBUD_LIBRARIES_DIRECTORY "c:/Program Files (x86)/IntelSWTools/compilers_and_libraries_2017.4.210/windows/compiler/lib/intel64_win")
  message("For the debuging on windows platform may required an additional libraries (e.g. ifconsole.lib).")
  message("Please add the library path to your project (e.g. 'c:/Program Files (x86)/IntelSWTools/compilers_and_libraries_2017.4.210/windows/compiler/lib/intel64_win')")
endif()


# -----------------------------------------------------------------------------
# UNIT TEST macro
# -----------------------------------------------------------------------------
macro(UNIT_TEST NAMESPACE NAME EXTRA_LIBS)
  if(UNC_BUILD_TESTS)
    add_executable(${NAMESPACE}_test_${NAME} ${NAME}_test.cpp)

    set_property(TARGET ${NAMESPACE}_test_${NAME} PROPERTY FOLDER uncertaintyTE/test)

    target_link_libraries(${NAMESPACE}_test_${NAME}
                          ${EXTRA_LIBS} # Extra libs MUST be first.
                          ${BOOST_LIBRARIES} uncertainty_shared)

  target_include_directories(${NAMESPACE}_test_${NAME}
        PUBLIC
        .
        ${INCLUDE_DIRS})

    add_test(NAME ${NAMESPACE}_test_${NAME}
       WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
       COMMAND $<TARGET_file:${NAMESPACE}_test_${NAME}> --log_level=all)
  endif()
endmacro()

# -----------------------------------------------------------------------------
# STATUS
# -----------------------------------------------------------------------------
message(STATUS " ")
message(STATUS "---------------------------------------------------------")
message(STATUS "USfM_USE_ALICEVISION: ${USfM_USE_ALICEVISION}")
message(STATUS "USfM_MKL_SUPPORT: ${USfM_MKL_SUPPORT}")
message(STATUS "USfM_BUILD_MEX : ${USfM_BUILD_MEX}")
message(STATUS "USfM_BUILD_TESTS: ${USfM_BUILD_TESTS}")
message(STATUS "---------------------------------------------------------")




## -----------------------------------------------------------------------------
## DEBUG OF CMAKE - PRINT ALL VARIABLES
## -----------------------------------------------------------------------------
#get_cmake_property(_variableNames VARIABLES)
#list (SORT _variableNames)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()