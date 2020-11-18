cmake_minimum_required(VERSION 3.15)
project(formation_1)

set(CMAKE_CXX_STANDARD 14)

include_directories("/home/lh/Desktop/OOQP/include")

link_directories("/home/lh/Desktop/OOQP/lib")

include_directories("/home/lh/eigen-3.3.7")

add_executable(formation_1 main.cpp formation.cpp MPC_p2p.cpp formationMain.cpp formationMain.h)

target_link_libraries(formation_1 ooqpgensparse
        ooqpsparse
        ooqpgondzio
        ooqpbase blas ma27 gfortran
        )