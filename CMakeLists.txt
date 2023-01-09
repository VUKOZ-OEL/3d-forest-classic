cmake_minimum_required (VERSION 3.1.0)
project(3DForest06)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
#Bring the headers, such as Student.h into the project
include_directories(sourceCode)
include_directories(images)


#However, the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "sourceCode/*.cpp")

#Can manually add the sources using the set command as follows:
#set(SOURCES sourceCode/3DForest.cpp)

ADD_EXECUTABLE(3DForest ${SOURCES} sourceCode/3dforest.qrc)

#dependencies - Qt
find_package(Qt5Widgets REQUIRED)
include_directories(${Qt5Widgets_INCLUDE_DIRS})
add_definitions(${Qt5Widgets_DEFINITIONS})
target_link_libraries(3DForest Qt5::Widgets)

#dependencies -libLAS
find_package(libLAS REQUIRED)
include_directories(${libLAS_INCLUDE_DIRS})
target_link_libraries(3DForest ${libLAS_LIBRARIES})

#dependencies - VTK
find_package(VTK 8.1 REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
target_link_libraries(3DForest ${VTK_LIBRARIES} )
LINK_DIRECTORIES(${VTK_LIB_DIRS})

#dependencies - PCL
find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PCL_BINARY_DIRS})
target_link_libraries(3DForest ${PCL_LIBRARIES})

#dependencies - Boost
find_package(Boost 1.72 COMPONENTS program_options thread system REQUIRED)
include_directories( ${Boost_INCLUDE_DIR} )
target_link_libraries(3DForest ${Boost_PROGRAM_OPTIONS_LIBRARY})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


#copy images into
#file(COPY images DESTINATION ${CMAKE_INSTALL_PREFIX})
#file(COPY sourceCode/projects.txt DESTINATION ${CMAKE_INSTALL_PREFIX})
#add_custom_command(TARGET 3DForest POST_BUILD COMMAND ${CMAKE_COMMAND} -E #copy_if_different sourceCode/projects.txt ${CMAKE_INSTALL_PREFIX})


