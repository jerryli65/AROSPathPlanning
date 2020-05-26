#include "aros_path_planning/library.hpp"

#include <iostream>

namespace aros_path_planning{
    void hello(){
        std::cout << "oof" << std::endl;
    }
}

#ifdef AROS_PATH_PLANNING_TEST

#include "gtest/gtest.h"

TEST(LibraryTest, EqualityTest){
#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCSimplifyInspection"
    ASSERT_TRUE(2 == 2);
#pragma clang diagnostic pop
    ASSERT_FALSE(2 + 2 == 5);
}

TEST(LibraryTest, HelloTest){
    aros_path_planning::hello();
}

#endif
