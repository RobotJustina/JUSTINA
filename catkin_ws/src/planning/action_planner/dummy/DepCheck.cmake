cmake_minimum_required(VERSION 2.8.3)

###
### Dependencies
###
set(
	ACTION_PLANER_DEP
	"arms"
	"base_ctrl"
	"bbros_bridge"
	"head"
	"language_understanding"
	"mvn_pln"
	"roah_rsbb_comm_ros"
)

foreach(DEP ${ACTION_PLANER_DEP})
	message(STATUS "Checking required package ${DEP}")
	set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dummy/${DEP}")
	set(TARGET_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../${DEP}")

	if( (NOT EXISTS "${TARGET_DIR}/CMakeLists.txt") AND (NOT EXISTS "${TARGET_DIR}/package.xml") )
		file(COPY "${SOURCE_DIR}" DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/../")
		message(WARNING "Package ${DEP} not found, dummy generated.")
		set(DEP_UPDATED TRUE)
	endif()
	
	unset(TARGET_DIR)
	unset(SOURCE_DIR)
endforeach(DEP)

if(DEP_UPDATED)
message(FATAL_ERROR "Some dependencies were not found. Dummies generated. Please run catkin again.")
endif()

unset(ACTION_PLANER_DEP)
