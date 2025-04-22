cmake_minimum_required(VERSION 3.21)
project(PMYproject)

set(CMAKE_CXX_STANDARD 17)
set(PACKAGE_NAME PMYproject)

# —————————————————————————————
# 1) External dependencies
# —————————————————————————————
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

# —————————————————————————————
# 2) 输出目录：所有可执行文件放到 bin/
# —————————————————————————————
# 在 configure 时，若 bin/ 不存在会自动创建
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)

# —————————————————————————————
# 3) Build qpOASES from source
# —————————————————————————————
file(GLOB QPOASES_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/PathTracking/utils/qpOASES/src/*.cpp"
)
add_library(qpOASES STATIC ${QPOASES_SRC})
target_include_directories(qpOASES PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/PathTracking/utils/qpOASES/include
)

# —————————————————————————————
# 4) Your core robotics library
# —————————————————————————————
add_library(PMYproject STATIC
    PathTracking/utils/MyReferencePath.cpp
    PathTracking/utils/KinematicModel.cpp
    PathTracking/utils/LowPassFilter.cpp
)
target_include_directories(PMYproject PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/PathTracking/utils
)
target_link_libraries(PMYproject PUBLIC
    Eigen3::Eigen
    qpOASES
    Python3::Python
    Python3::Module
    Python3::NumPy
)

# —————————————————————————————
# 6) LQR demo
# —————————————————————————————
add_executable(lqr_demo
    PathTracking/LQR/main.cpp
    PathTracking/LQR/LQRControl.cpp
)
target_include_directories(lqr_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/PathTracking/LQR
    ${CMAKE_CURRENT_SOURCE_DIR}/PathTracking/utils
)
target_link_libraries(lqr_demo PRIVATE PMYproject)

# —————————————————————————————
# 7) MPC demo
# —————————————————————————————
add_executable(mpc_demo
    PathTracking/MPC/MpcMain.cpp
    PathTracking/MPC/Mpc.cpp
)
target_include_directories(mpc_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/PathTracking/MPC
    ${CMAKE_CURRENT_SOURCE_DIR}/PathTracking/utils
)
target_link_libraries(mpc_demo PRIVATE PMYproject)
