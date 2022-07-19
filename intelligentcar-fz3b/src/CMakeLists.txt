cmake_minimum_required(VERSION 3.4...3.18)
project(intelligentCar)

set (CMAKE_CXX_STANDARD 17)


#---------------------------------------------------------------------
#       [ Include、Lib  : Define ] ==> []
#---------------------------------------------------------------------

set(COMMON_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/")

set(COMMON_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include"
                        "${PROJECT_SOURCE_DIR}/config"
                        "${PROJECT_SOURCE_DIR}/demo" 
                        "${PROJECT_SOURCE_DIR}/driver" 
                        "${PROJECT_SOURCE_DIR}/lib" 
                        "${PROJECT_SOURCE_DIR}/src"
                        "${PROJECT_SOURCE_DIR}/tool"          )

link_directories(${COMMON_LIB_DIR})
include_directories( ${COMMON_INCLUDE_DIR} )

#---------------------------------------------------------------------
#       [ Include、Lib  : Path ] ==> [ glib opencv ]
#---------------------------------------------------------------------
find_package(PkgConfig)
pkg_search_module(GLIB REQUIRED glib-2.0)
include_directories(${GLIB_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )    
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(PaddleLite QUIET)
include_directories(${PADDLELITE_INCLUDE_DIR})
LINK_DIRECTORIES("/usr/local/lib/paddle_lite/")
message(${PADDLELITE_LIBRARY})
message("Install Prefix : [${CMAKE_INSTALL_PREFIX}]")

#---------------------------------------------------------------------
#               [ bin ] ==> [ tool ]
#---------------------------------------------------------------------
# Image2video
set(IMAGE2VIDEO_PROJECT_NAME "image2video")
set(IMAGE2VIDEO_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tool/image2video.cpp)
add_executable(${IMAGE2VIDEO_PROJECT_NAME} ${IMAGE2VIDEO_PROJECT_SOURCES})
target_link_libraries(${IMAGE2VIDEO_PROJECT_NAME} PRIVATE pthread )
target_link_libraries(${IMAGE2VIDEO_PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

# CameraDisplay
set(CAMERA_DISPLAY_PROJECT_NAME "camera_display")
set(CAMERA_DISPLAY_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tool/camera_display.cpp)
add_executable(${CAMERA_DISPLAY_PROJECT_NAME} ${CAMERA_DISPLAY_PROJECT_SOURCES})
target_link_libraries(${CAMERA_DISPLAY_PROJECT_NAME} PRIVATE pthread )
target_link_libraries(${CAMERA_DISPLAY_PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

# ImageCollection
set(COLLECTION_PROJECT_NAME "image_collection")
set(COLLECTION_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tool/image_collection.cpp)
add_executable(${COLLECTION_PROJECT_NAME} ${COLLECTION_PROJECT_SOURCES})
target_link_libraries(${COLLECTION_PROJECT_NAME} PRIVATE pthread )
target_link_libraries(${COLLECTION_PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
target_link_libraries(${COLLECTION_PROJECT_NAME} PRIVATE serial)

#---------------------------------------------------------------------
#               [ bin ] ==> [ main ]
#---------------------------------------------------------------------
set(PROJECT_NAME "icar")
set(INTELLIGENTCAR_CAR_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/src/icar.cpp)
add_executable(${PROJECT_NAME} ${INTELLIGENTCAR_CAR_PROJECT_SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE pthread )
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} PRIVATE paddle_full_api_shared)
target_link_libraries(${PROJECT_NAME} PRIVATE serial)

install(TARGETS ${PROJECT_NAME} 
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX})


