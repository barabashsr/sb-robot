cmake_minimum_required(VERSION 3.10)
project(test)

find_package(WebSocketPP REQUIRED)
find_package(OpenSSL REQUIRED)



add_executable(${PROJECT_NAME} src/main.cpp src/ws_driver.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries(${PROJECT_NAME} ${WebSocketPP_LIBRARIES} ${OPENSSL_LIBRARIES})

