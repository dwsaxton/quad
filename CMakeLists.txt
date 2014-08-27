project(Quad)
cmake_minimum_required(VERSION 2.6)
find_package(Qt4 COMPONENTS QtOpenGL REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR} /usr/include/eigen3)

SET(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_AUTOMOC ON)
set(Quad_SRCS hw/raspberryi2c.cpp hw/hmc5883.cpp hw/itg3200.cpp hw/adxl345.cpp propellers.cpp memsimu.cpp imu.cpp globals.cpp onlinelearner.cpp planner1d.cpp linearplannermono.cpp ssc.cpp pathinterceptplanner.cpp linearplanner3d.cpp interceptplot.cpp linearplanner.cpp quadratic3d.cpp quadraticintercept.cpp rotationplanner.cpp highercontroller.cpp simplequadraticintercept.cpp sensors.cpp observer.cpp interface.cpp qpsolver.cpp quadstate.cpp quad.cpp controllooper.cpp observer.cpp controller.cpp main.cpp glquad.cpp)
set(Quad_FORMS interface.ui)
QT4_WRAP_UI(Quad_FORMS_HEADERS ${Quad_FORMS})
qt4_automoc(${Quad_SRCS})
add_executable(Quad ${Quad_SRCS} ${Quad_FORMS_HEADERS})
target_link_libraries(Quad ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${OPENGL_LIBRARIES})
install(TARGETS Quad RUNTIME DESTINATION bin)

add_subdirectory(hw)