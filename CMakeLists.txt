cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Mobility_Map_builder)

find_package(PCL 1.7 REQUIRED)

find_package(OpenCV REQUIRED)

find_package(CGAL QUIET COMPONENTS Core Qt4 )

if ( CGAL_FOUND )

  include( ${CGAL_USE_FILE} )

  find_package(Qt4-patched QUIET )
  # FindQt3-patched.cmake is FindQt3.cmake patched by CGAL developers, so
  # that it can be used together with FindQt4: all its variables are prefixed
  # by "QT3_" instead of "QT_".

  if(CGAL_Qt4_FOUND AND QT4_FOUND)

    include( Qt3Macros-patched )
    qt4_automoc(  main.cpp )

    # Make sure the compiler can find generated .moc files
    include_directories(BEFORE ${CMAKE_CURRENT_BINARY_DIR})

    include_directories( ${QT4_INCLUDE_DIR} )
    endif()
    endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PCL_BUILD_TYPE Release)
set(SOURCE_FILES main.cpp)

add_executable(mobilityMapBuilder ${SOURCE_FILES} ComputeArea.h ComputeArea.cpp dot.h dot.cpp MobilityScore.h MobilityScore.cpp)

target_link_libraries (mobilityMapBuilder ${PCL_LIBRARIES}${QT4_LIBRARIES} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${OpenCV_LIBS})
