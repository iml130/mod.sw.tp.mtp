//  Copyright 2020 Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  https:www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#include "mars_common/Id.h"

#include <gtest/gtest.h>

TEST(IdTests, uninitialized)
{
  mars::common::Id lID;

  EXPECT_NO_THROW(lID.isInitialized());
  EXPECT_FALSE(lID.isInitialized());

  EXPECT_THROW(lID.getUUID(), mars::common::exception::NotInitializedException);
  EXPECT_THROW(lID.getUUIDAsString(), mars::common::exception::NotInitializedException);
}

TEST(IdTests, default_initialization)
{
  mars::common::Id lID;

  EXPECT_NO_THROW(lID.initialize());
  EXPECT_TRUE(lID.isInitialized());

  EXPECT_NO_THROW(lID.getDescription());
  EXPECT_NO_THROW(lID.getUUID());
  EXPECT_NO_THROW(lID.getUUIDAsString());
}

TEST(IdTests, clone_construction)
{
  mars::common::Id lID;
  lID.initialize();

  EXPECT_NO_THROW(mars::common::Id(lID.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT),
                                   lID.getDescription()));
  EXPECT_NO_THROW(mars::common::Id(lID.getUUIDAsString(mars::common::Id::UUIDFormat::BYTEWISE),
                                   lID.getDescription()));
  EXPECT_NO_THROW(mars::common::Id(lID.getUUID(), lID.getDescription()));
  EXPECT_NO_THROW(mars::common::Id(lID));

  EXPECT_EQ(lID.getUUIDAsString(mars::common::Id::UUIDFormat::BYTEWISE),
            mars::common::Id(lID.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT),
                             lID.getDescription())
                .getUUIDAsString(mars::common::Id::UUIDFormat::BYTEWISE));
  EXPECT_EQ(lID.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC),
            mars::common::Id(lID.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT),
                             lID.getDescription())
                .getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC));
  EXPECT_EQ(lID.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT),
            mars::common::Id(lID.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC),
                             lID.getDescription())
                .getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT));
}

TEST(IdTests, copy_initialization)
{
  mars::common::Id lID1;
  mars::common::Id lID2;
  mars::common::Id lID3;
  mars::common::Id lID4;
  mars::common::Id lID5;

  lID1.initialize();

  EXPECT_NO_THROW(lID2.initialize(lID1.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT), lID1.getDescription()));
  EXPECT_NO_THROW(lID3.initialize(lID1.getUUIDAsString(mars::common::Id::UUIDFormat::BYTEWISE), lID1.getDescription()));
  EXPECT_NO_THROW(lID4.initialize(lID1.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC), lID1.getDescription()));
  EXPECT_NO_THROW(lID5.initialize(lID1.getUUID(), lID1.getDescription()));
}

TEST(IdTests, copy_construction_from_message)
{
  mars::common::Id lID;

  std::string lDescription = "MARS::Common::ID::Description::LongString TEST";

  lID.initialize();
  EXPECT_NO_THROW(lID.setDescription(lDescription));

  mars_common_msgs::Id lIdMsg;
  lIdMsg.description = lID.getDescription();
  lIdMsg.uuid = lID.getUUID();

  mars::common::Id lIDCopy;
  EXPECT_NO_THROW(lIDCopy = mars::common::Id(lIdMsg));

  EXPECT_EQ(lIDCopy.getDescription(), lDescription);
  EXPECT_EQ(lIDCopy.getUUID(), lID.getUUID());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}