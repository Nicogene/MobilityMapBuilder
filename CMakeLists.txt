cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Mobility_Map_builder)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(PCL 1.7 REQUIRED)

find_package(OpenCV REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

#find_package(Eigen REQUIRED)

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

include_directories(${PCL_INCLUDE_DIRS}) #${EIGEN_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PCL_BUILD_TYPE Release)
file(GLOB Executable_SOURCES *.cpp)
file(GLOB Executable_HEADERS *.h)

add_executable(mobilityMapBuilder ${Executable_SOURCES})

target_link_libraries (mobilityMapBuilder ${PCL_LIBRARIES}${QT4_LIBRARIES} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${OpenCV_LIBS})

if(${CMAKE_VERSION} VERSION_LESS 3.1)
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
  check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
else()
  target_compile_features(mobilityMapBuilder PRIVATE cxx_range_for)
endif()
