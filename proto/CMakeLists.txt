
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../Middlewares/Third_Party/nanopb/extra)

find_package(Nanopb REQUIRED)

FILE(GLOB PROTOFILES "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
include_directories(${NANOPB_INCLUDE_DIRS})
nanopb_generate_cpp(PROTO_SRCS PROTO_HEADERS ${PROTOFILES})

add_library(nanopb ${PROTO_HEADERS} ${PROTO_SRCS})
target_link_libraries(nanopb PUBLIC )
target_include_directories(nanopb INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/pb>)