set(
        AROS_PATH_PLANNING_SRC
        library.cpp
        Odometry.cpp
)

add_library(AROS_PATH_PLANNING STATIC ${AROS_PATH_PLANNING_SRC})
target_include_directories(AROS_PATH_PLANNING PUBLIC ${PROJECT_SOURCE_DIR}/include)

if(AROS_PATH_PLANNING_TESTING)
    set(
            AROS_PATH_PLANNING_TEST_SRC
            Odometry.cpp)
    add_executable(AROS_PATH_PLANNING_TEST ${AROS_PATH_PLANNING_SRC} ${AROS_PATH_PLANNING_TEST_SRC})
    target_include_directories(AROS_PATH_PLANNING_TEST PUBLIC ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(AROS_PATH_PLANNING_TEST gtest gtest_main)
    target_compile_definitions(AROS_PATH_PLANNING_TEST PRIVATE AROS_PATH_PLANNING_TEST)
endif()
