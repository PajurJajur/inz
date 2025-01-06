cmake_minimum_required(VERSION 3.10)

# Nazwa projektu
project(LuxonisExample)
set(depthai_DIR "/home/PajurJajur/Desktop/depthai-core/build")
find_package(depthai CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
set(BLOB_PATH "/home/PajurJajur/Desktop/integrating_sensors/build/Cone.blob")

include_directories(
    "/home/PajurJajur/Desktop/integrating_sensors" # Ścieżka do rotary_encoder.hpp i czujniki.cpp
    "/home/PajurJajur/Desktop/integrating_sensors/t34" # Ścieżka do pid_control.h
)
# Dodanie pliku źródłowego (main.cpp)
add_executable(main main.cpp rotary_encoder.cpp)


# Powiązanie biblioteki DepthAI (np. depthai::opencv, jeśli używasz OpenCV)
target_link_libraries(main PRIVATE depthai::opencv ${OpenCV_LIBS})
target_link_libraries(main PRIVATE pthread pigpio wiringPi) # Używane biblioteki GPIO
target_compile_definitions(main PRIVATE BLOB_PATH="${BLOB_PATH}")