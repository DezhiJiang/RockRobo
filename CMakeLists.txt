cmake_minimum_required(VERSION 2.8)
#之后也需要修改扫地机器人上的相应配置文件为对应名字
project(RockroboBridge)

#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_PREFIX_PATH "/home/lzy/google_carto_ws/src/cartographer/cmake")
#find_package(cartographer REQUIRED)
#include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-reorder -O2 -DRRLOG")

#修改CMAKE_BUILD_TYPE  之后在Democode中有设置
set(CMAKE_BUILD_TYPE Release)

#两种类型的文件需用引号分开
file(GLOB_RECURSE ALL_SRCS "src/*.cpp" "src/*.h")

include_directories(/home/lzy/LiteDriver/DemoCode/include)


#MESSAGE(${CMAKE_SYSTEM_NAME})

#用于编译player中程序
#MESSAGE(${CMAKE_CURRENT_LIST_DIR})
add_library(litedriverdef INTERFACE)
target_compile_features(litedriverdef INTERFACE cxx_auto_type)
target_compile_definitions(litedriverdef INTERFACE _USE_RRLOG)
target_include_directories(litedriverdef INTERFACE /home/lzy/LiteDriver/toolchain/rrlog/arm/include)
target_link_libraries(litedriverdef INTERFACE /home/lzy/LiteDriver/toolchain/rrlog/arm/lib/librrlog.so)
target_include_directories(litedriverdef INTERFACE /home/lzy/LiteDriver/toolchain/player/arm/include/player-3.1)
#MESSAGE(${CMAKE_ROOT})
#MESSAGE(${CMAKE_SYSTEM_NAME})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)
set(CMAKE_MODULE_PATH /home/lzy/car_cc/share/cartographer/cmake/modules)

set(cartographer_DIR /home/lzy/car_cc/share/cartographer/)
find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
set(BUILD_SHARED_LIBS OFF)
google_initialize_cartographer_project()
google_enable_testing()

include(FindPkgConfig)
set(Boost_ADDITIONAL_VERSIONS "1.55" "1.55.0")
set(BOOST_ROOT "/home/lzy/car_cc/boost_1.55")
set(BOOST_INCLUDEDIR "/home/lzy/car_cc/boost_1.55/include")
set(BOOST_LIBRARYDIR "/home/lzy/car_cc/boost_1.55/arm/arm-linux-gnueabihf")
set(LUA_INCLUDE_DIR "/home/lzy/car_cc/include")
set(LUA_LIBRARIES "/home/lzy/car_cc/lib/liblua.so")
set(LUA_DIR "/home/lzy/car_cc")

find_package(LuaGoogle REQUIRED)
#find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)


INCLUDE_DIRECTORIES(/usr/include/eigen3)
INCLUDE_DIRECTORIES(/home/lzy/google_carto_ws/src/cartographer)
#INCLUDE_DIRECTORIES(/home/lzy/google_carto_ws/build_isolated/cartographer/install)
INCLUDE_DIRECTORIES(/home/lzy/LiteDriver/toolchain/player/arm/include/player-3.1)

#给librrlog添加的路径
INCLUDE_DIRECTORIES(/home/lzy/LiteDriver/toolchain/rrlog/arm/include)


#动态库  仿照player的Democode
add_library(${PROJECT_NAME} SHARED ${ALL_SRCS})
target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)
target_link_libraries(${PROJECT_NAME} PUBLIC litedriverdef ${LIB_CONFIG})

#使用SYSTEM 添加的库之后需要交叉编译修改
#Lua
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})

#PCL    这个库是否可以不需要
#target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
#foreach(DEFIMTION ${PCL_DEFINITIONS})
#    set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
#endforeach()

#Eigen
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC -lglog)

#先添加二进制目录，然后port.h能在生成后被添加
#target_include_directories(${PROJECT_NAME} PUBLIC
#        $<BUILD_INTERFACE:${RPOJECT_BINARY_DIR}>
#        $<BUILD_INTERFACE:${RPOJECT_SOURCE_DIR}>
#        $<INSTALL_INTERFACE:include>
#        )

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
#设置compile_flags
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})



