# Add unit test
if(UNC_BUILD_TESTS)
	IF (NOT UNC_BUILD_SHARED_LIB)
		MESSAGE(WARNING "You have to build share library to enable testing ... ")
	ELSE()
	
		# List of unit tests
		UNIT_TEST(cov covarianceEigenValues "uncertainty_shared")
		
		
		
	ENDIF()
ENDIF()