cmake_minimum_required(VERSION 2.8.3)
project(unity_bridge)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp
	rospy
	tf
	std_msgs
  mav_msgs
)

catkin_package(CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  geometry_msgs 
  mav_msgs 
  tf
)

SET(TEST_FLAG 0;1 CACHE BOOLEAN 1)

include(ExternalProject)
if(TEST_FLAG)
  ExternalProject_Add(unitysim
      URL               https://www.dropbox.com/s/t7sphxie0yyrgjv/AS_Data.zip?dl=0
      DOWNLOAD_NAME     AS_Data.zip
      #URL_HASH					MD5=e7de46d5197e182cf53b6f462b8956f4
      CONFIGURE_COMMAND chmod +x ../unitysim/VNAV.x86_64
      BUILD_COMMAND     ""
      INSTALL_COMMAND   cd ../unitysim/ && cp -rf . ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  ExternalProject_Add(unitysim
  URL               https://www.dropbox.com/s/si54wh06udzpbd0/Linux_build_server_Data.zip?dl=0
  DOWNLOAD_NAME     Linux_build_server_Data.zip
  #URL_HASH					MD5=e7de46d5197e182cf53b6f462b8956f4
  CONFIGURE_COMMAND chmod +x ../unitysim/Linux_build_server.x86_64
  BUILD_COMMAND     ""
  INSTALL_COMMAND   cd ../unitysim/ && cp -rf . ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

ExternalProject_Get_Property(unitysim DOWNLOAD_DIR)
set_property(DIRECTORY PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/."
)

message(STATUS "UNITYSIM DOWNLOAD_DIR: ${DOWNLOAD_DIR}")

ExternalProject_Add(libsocket
  GIT_REPOSITORY https://github.com/dermesser/libsocket
  UPDATE_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX} -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_SHARED_LIBS=ON
  BUILD_COMMAND make
  INSTALL_COMMAND make install
)

find_package(Eigen3)

link_directories(${CATKIN_DEVEL_PREFIX}/lib)
include_directories(${CATKIN_DEVEL_PREFIX}/include)
include_directories(${catkin_INCLUDE_DIRS} tcpimage)

add_subdirectory(tcpimage)

add_executable(w_to_unity src/w_to_unity.cpp)
target_link_libraries(w_to_unity ${catkin_LIBRARIES} socket++ -pthread)
add_dependencies(w_to_unity libsocket ${catkin_EXPORTED_TARGETS})

add_executable(unity_state src/unity_state.cpp)
add_dependencies(unity_state ${catkin_EXPORTED_TARGETS})
target_link_libraries(unity_state ${catkin_LIBRARIES} tcpstreamreader)
