cmake_minimum_required(VERSION 2.8.3)

###
### Dependencies
###
set(
	TASK_PLANER_DEP
	"bbros_bridge"
    "language_understanding"
	"hri"
	"planning"
)

foreach(DEP ${TASK_PLANER_DEP})
	message(STATUS "Checking required package ${DEP}")
	set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dummy/${DEP}")
	set(TARGET_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../${DEP}")

    #if( (NOT EXISTS "${TARGET_DIR}/CMakeLists.txt") AND (NOT EXISTS "${TARGET_DIR}/package.xml") )
    if( NOT EXISTS "${TARGET_DIR}")
		file(COPY "${SOURCE_DIR}" DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/../")
		message(WARNING "Package ${DEP} not found, dummy generated.")
		set(DEP_UPDATED TRUE)
	endif()
	
	unset(TARGET_DIR)
	unset(SOURCE_DIR)
endforeach(DEP)

if(DEP_UPDATED)
message(FATAL_ERROR "Some dependencies were not found. Dummies generated. Please run catkin_make again.")
endif()

unset(TASK_PLANER_DEP)
