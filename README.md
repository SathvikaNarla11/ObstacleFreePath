## Requirements

- C++17 or later
- OpenCV via vcpkg
- CMake ≥ 3.10

## Install openCV
- git clone https://github.com/microsoft/vcpkg.git
- ./vcpkg/bootstrap-vcpkg.bat
- ./vcpkg install opencv4


## Build Instructions
- Clone the project
    - git clone https://github.com/SathvikaNarla11/ObstacleFreePath.git
- cd <go-to-folder>
- Integrate vcpkg with CMake (only needs to be done once)  
    - ./vcpkg integrate install
- In teminal
- mkdir build
- cd build
- cmake .. -DCMAKE_TOOLCHAIN_FILE=<path/to/vcpkg/scripts/buildsystems/vcpkg.cmake>
- cmake --build .
- cd debug/RRTGrid.exe //adjust if executable path is different



