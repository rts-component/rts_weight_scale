cmake_minimum_required(VERSION 3.16)

project(rts_weight_scale
	VERSION 1.0.0
	LANGUAGES C
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(
	unity
	GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
	GIT_TAG v2.6.1
	GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(unity)

add_executable(${PROJECT_NAME}
	test_main.c
	../rts_weight_scale.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
	..
)

target_link_libraries(${PROJECT_NAME} PRIVATE unity)

enable_testing()
add_test(NAME ${PROJECT_NAME}_tests COMMAND ${PROJECT_NAME})
