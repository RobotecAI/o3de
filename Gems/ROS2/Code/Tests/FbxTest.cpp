/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Fbx.h>

#include <AzTest/AzTest.h>

namespace {

class FbxTest : public ::testing::Test
{
    public:

    protected:
        ROS2::Fbx fbx; 
};

TEST_F(FbxTest, BasicStructureGeneration)
{
    const auto fbxStr = fbx.ToString();

    std::istringstream iss(fbxStr);
    std::string line;

    std::getline(iss, line)
    EXPECT_STR_EQ(line, "FBXHeaderExtension:  {");

    std::getline(iss, line)
    EXPECT_STR_EQ(line, "	FBXHeaderVersion: 1003");

    std::getline(iss, line)
    EXPECT_STR_EQ(line, "	FBXVersion: 7500");
}

} // namespace
