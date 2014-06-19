project(Quad)
cmake_minimum_required(VERSION 2.6)
find_package(Qt4 REQUIRED)

include_directories(${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})

set(Quad_SRCS Quad.cpp main.cpp)
qt4_automoc(${Quad_SRCS})
add_executable(Quad ${Quad_SRCS})
target_link_libraries(Quad ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
install(TARGETS Quad RUNTIME DESTINATION bin)
