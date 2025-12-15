
cmake_minimum_required(VERSION 3.16.0)

add_executable(cwipc_orbbec_grab cwipc_orbbec_grab.cpp)
include_directories(
	"../../include"
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(cwipc_orbbec_grab cwipc_orbbec cwipc_util)

# install(TARGETS cwipc_orbbec_grab RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_test(NAME cwipc_orbbec_grab COMMAND cwipc_orbbec_grab 2 . )
set_tests_properties(cwipc_orbbec_grab PROPERTIES ENVIRONMENT CWIPC_ORBBEC_TESTING=yes)
