cmake_minimum_required(VERSION 3.0.2)
project(Lower_Limb_Exoskeleton)

find_package(Boost REQUIRED COMPONENTS system thread regex)

include_directories(
    Inc
    ${Boost_INCLUDE_DIR}
)

set(SRC
    ${CMAKE_CURRENT_LIST_DIR}/Src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/Src/crc32_mpeg.cpp
    ${CMAKE_CURRENT_LIST_DIR}/Src/communication_embedded_system.cpp
    ${CMAKE_CURRENT_LIST_DIR}/Src/exoskeleton.cpp
    ${CMAKE_CURRENT_LIST_DIR}/Src/lsa.cpp
)

set(LIBS
    ${Boost_LIBRARIES}
)

add_executable(exoskeleton ${SRC})
target_link_libraries(exoskeleton ${LIBS})
