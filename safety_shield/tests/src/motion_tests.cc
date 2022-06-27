#include <vector>

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "motion_fixture.h"
#include "safety_shield/motion.h"

namespace safety_shield {

TEST_F(MotionTestSimple, SimpleGetTest){
  EXPECT_DOUBLE_EQ(motion_.getTime(), 0.0);
  EXPECT_DOUBLE_EQ(motion_.getS(), 0.0);
  EXPECT_EQ(motion_.getNbModules(), 3);
}

} // namespace safety_shield

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}