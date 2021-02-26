#include "mars_common/TimeInterval.h"

#include <gtest/gtest.h>

TEST(TimeIntervalTests, initialization)
{
  ros::Time lValidStartTime(0);
  ros::Duration lValidDuration(1);
  ros::Duration lInvalidDuration(0);

  EXPECT_THROW(mars::common::TimeInterval(lValidStartTime, lInvalidDuration), mars::common::exception::SetParamException);
  EXPECT_NO_THROW(mars::common::TimeInterval(lValidStartTime, lValidDuration));
}

TEST(TimeIntervalTests, copy_construction_from_msg)
{
  ros::Time lValidStartTime(777);
  ros::Duration lValidDuration(1337);

  mars::common::TimeInterval* lTimeInterval;
  mars::common::TimeInterval* lTimeIntervalCopy;

  EXPECT_NO_THROW(lTimeInterval = new mars::common::TimeInterval(lValidStartTime, lValidDuration));

  mars_topology_msgs::TimeInterval lTimeIntervalMsg;
  EXPECT_NO_THROW(lTimeIntervalMsg = lTimeInterval->toMsg());
  EXPECT_NO_THROW(lTimeIntervalCopy = new mars::common::TimeInterval(lTimeIntervalMsg));
  EXPECT_EQ(lTimeInterval->getStartTime(), lValidStartTime);
  EXPECT_EQ(lTimeIntervalCopy->getStartTime(), lValidStartTime);
  EXPECT_EQ(lTimeInterval->getDuration(), lValidDuration);
  EXPECT_EQ(lTimeIntervalCopy->getDuration(), lValidDuration);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}