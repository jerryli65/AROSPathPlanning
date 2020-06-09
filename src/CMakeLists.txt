set(
        AROS_PATH_PLANNING_SRC
        Odometry.cpp
        PositionTracking.cpp
        PathPlanning.cpp
)

add_library(AROS_PATH_PLANNING STATIC ${AROS_PATH_PLANNING_SRC})
target_include_directories(AROS_PATH_PLANNING PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(AROS_PATH_PLANNING AROS)

if(AROS_PATH_PLANNING_TESTING)
    set(
            AROS_PATH_PLANNING_TEST_SRC
    )
    add_executable(AROS_PATH_PLANNING_TEST ${AROS_PATH_PLANNING_SRC} ${AROS_PATH_PLANNING_TEST_SRC})
    target_include_directories(AROS_PATH_PLANNING_TEST PUBLIC ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(AROS_PATH_PLANNING_TEST AROS gtest gtest_main)
    target_compile_definitions(AROS_PATH_PLANNING_TEST PRIVATE AROS_PATH_PLANNING_TEST)
endif()
