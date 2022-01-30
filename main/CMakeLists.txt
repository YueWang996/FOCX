# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

file(GLOB DRIVER driver/*.cpp)
file(GLOB MAINCPP *.cpp)

set(COMPONENT_SRCS ${MAINCPP} ${DRIVER})
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()
