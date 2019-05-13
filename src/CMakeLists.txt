# -----------------------------------------------------------------------------
# Set sources and create executable target
# -----------------------------------------------------------------------------

file(GLOB MODUL_DEFINITION_FILES "*.def")
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)


# ==============================================================================
# Uncertainty Library
# ==============================================================================

file(GLOB USfM_HEADERS "USfM/*.hpp")
file(GLOB USfM_SOURCES "USfM/*.cpp")

add_library(usfm SHARED ${USfM_HEADERS} ${USfM_SOURCES})
target_include_directories(usfm
  PUBLIC
	.
	"${CMAKE_SOURCE_DIR}/src"
    ${EIGEN3_INCLUDE_DIRS}
	${OPENBLAS_INCLUDE_DIRS}
	${CLAPACK_INCLUDE_DIRS}
    ${SUITESPARSE_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${Matlab_INCLUDE_DIRS} # optional
    ${MEX_INCLUDE_DIR} # optional
    ${Boost_INCLUDE_DIRS}
	${MKL_INCLUDE_DIR}
  )
target_link_libraries(usfm
	#${EIGEN3_INCLUDE_DIRS}
	${OPENBLAS_LIBRARIES}
	${CLAPACK_LIBRARIES}
	${SUITESPARSE_LIBRARIES}
	${CERES_LIBRARIES}
	${Matlab_LIBRARIES} # optional
	${Boost_LIBRARIES}
	${MKL_LIBRARIES}
  )
target_compile_definitions(usfm
  PUBLIC ${EIGEN_FLAGS}
  )
if(USfM_MKL_SUPPORT)
	add_definitions("/DMKL_ILP64")
endif()
install(TARGETS usfm
    DESTINATION lib
    EXPORT usfm-targets
  )

# ==============================================================================
# Uncertainty command line application
# ==============================================================================
add_executable(uncertainty software/usfm_main.cpp)
target_include_directories(uncertainty
    PUBLIC
    .
    ${INCLUDE_DIRS}
    ${GFLAGS_INCLUDE_DIR})

target_link_libraries(uncertainty
  usfm
  ${GFLAGS_LIBRARIES}
  )

install(DIRECTORY usfm
  DESTINATION include
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
  )


# ==============================================================================
# MATLAB extension
# ==============================================================================
if(USfM_BUILD_MEX)
  matlab_add_mex(
    NAME usfm_mex
    SRC matlab/usfm_mex.cpp
    LINK_TO usfm)
endif(USfM_BUILD_MEX)


# ==============================================================================
# UNIT TEST framework
# ==============================================================================
if(USfM_BUILD_TESTS)
  enable_testing()
endif()


# Tests 
add_definitions(-DTESTS_ROOT_DIR="${CMAKE_SOURCE_DIR}/src/test")
#add_subdirectory(test_eigValues)




