# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

file(GLOB DRIVER driver/*.cpp)
file(GLOB MAINCPP *.cpp)
file(GLOB TLE5012B sensor/Tle5012b/*.cpp)
file(GLOB FOC FOC/*.cpp)
#file(GLOB CAN_BUS_C driver/CAN/*.c)
#file(GLOB CAN_BUS_CPP driver/CAN/*.cpp)
#file(GLOB CAN_CONFIG driver/CAN/CAN_config.h)

#file(GLOB DRIVER_H driver/*.h)
#file(GLOB MAIN_H *.h)
#file(GLOB TLE5012B_H sensor/Tle5012b/*.h)

set(COMPONENT_SRCS ${MAINCPP} ${DRIVER} ${TLE5012B} ${FOC})# ${CAN_BUS_C} ${CAN_BUS_CPP} ${CAN_CONFIG})
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()
