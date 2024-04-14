cmake_minimum_required(VERSION 3.5)
project(ompl_tutorial)


#foreach(file /usr/lib/x86_64-linux-gnu/libOpenGL.so;/usr/lib/x86_64-linux-gnu/libGLX.so;/usr/lib/x86_64-linux-gnu/libGLU.so;/home/kenny)
#	if (EXISTS ${file})
#		set(FOO TRUE)
#		message("${file} FOUND")
#		message("TEST FOUND")
#	else()
#		message("${file} hahah")
#		set(FOO FALSE)
#	endif()
#endforeach()
#
#message(${FOO})


# search for ompl CMake module in format "FindXXX.cmake" from CMAKE_MODULE_PATH
find_package(ompl)

# check if package is found
if (OMPL_FOUND)
	message("ompl found")
else()
	message(FATAL_ERROR "CANNOT FIND Ompl")
endif()

# add_executable
add_executable(${PROJECT_NAME}_geom_planning src/geometricPlanningSE3.cpp)

# include ompl headers
target_include_directories(${PROJECT_NAME}_geom_planning 
	PRIVATE ${OMPL_INCLUDE_DIRS}
)

# link ompl libraries
target_link_libraries(${PROJECT_NAME}_geom_planning
	PRIVATE ${OMPL_LIBRARIES}
)
       
       	


