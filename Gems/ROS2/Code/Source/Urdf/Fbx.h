/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <string>
#include <any>

namespace ROS2
{
    class Fbx
    {
        public:
            using Property = std::any;
            using Properties = std::vector<Property>;
            enum class FileType { Text, Binary };

            class Node
            {
                public:
                    Node(const std::string & name, const Properties & properties = {})
                        : m_name(name), m_properties(properties)
                    {}

                    void AddProperty(const Property & property)
                    {
                        m_properties.push_back(property);
                    }

                    void AddChildNode(const Node & child)
                    {
                        m_children.push_back(child);
                    }

                    void AddChildNode(const std::string & name, const Property & property)
                    {
                        m_children.push_back(Node(name, { property }));
                    }

                    void AddChildNode(const Node && child)
                    {
                        m_children.push_back(child);
                    }

                private:
                    std::string m_name;
                    std::vector<Node> m_children;
                    Properties m_properties;
            };

            Node GetFbxHeaderExtension()
            {
                Node fbxHeader("FBXHeaderExtension");
                fbxHeader.AddChildNode("FBXHeaderVersion", 1003);
                fbxHeader.AddChildNode("FBXVersion", 7500);
                fbxHeader.AddChildNode(GetTimeStamp());
                fbxHeader.AddChildNode("Creator", "O3DE URDF->FBX Converter");
                fbxHeader.AddChildNode(GetSceneInfo());

                Node properties("Properties70");
                properties.AddChildNode(
                    Node("P", {"DocumentUrl", "KString", "Url", "", "/dummy_path.fbx"}));
                properties.AddChildNode(
                    Node("P", {"SrcDocumentUrl", "KString", "Url", "", "/dummy_path.fbx"}));
                properties.AddChildNode(
                    Node("P", {"Original", "Compound", "", ""}));
                properties.AddChildNode(
                    Node("P", {"Original|ApplicationVendor", "KString", "", "", "O3DE"}));
                properties.AddChildNode(
                    Node("P", {"Original|ApplicationName", "KString", "", "", "O3DE"}));
                properties.AddChildNode(
                    Node("P", {"Original|ApplicationVersion", "KString", "", "", "2022"}));
                properties.AddChildNode(
                    Node("P", {"Original|DateTime_GMT", "DateTime", "", "", "01/01/2022 00:00:00.000"}));
                properties.AddChildNode(
                    Node("P", {"Original|FileName", "KString", "", "", "/dummy_path.fbx"}));
                properties.AddChildNode(
                    Node("P", {"LastSaved", "Compound", "", ""}));
                properties.AddChildNode(
                    Node("P", {"LastSaved|ApplicationVendor", "KString", "", "", "O3DE"}));
                properties.AddChildNode(
                    Node("P", {"LastSaved|ApplicationName", "KString", "", "", "O3DE"}));
                properties.AddChildNode(
                    Node("P", {"LastSaved|ApplicationVersion", "KString", "", "", "2022"}));
                properties.AddChildNode(
                    Node("P", {"LastSaved|DateTime_GMT", "DateTime", "", "", "01/01/2022 00:00:00.000"}));
                properties.AddChildNode(
                    Node("P", {"Original|ApplicationActiveProject", "KString", "", "", "/dummy_path.fbx"}));
                properties.AddChildNode(
                    Node("P", {"Original|ApplicationNativeFile", "KString", "", "", "/dummy_path.fbx"}));

                fbxHeader.AddChildNode(std::move(properties));

                return fbxHeader;
            }

        private:
            Node GetTimeStamp()
            {
                // TODO: get proper time stamp
                Node timeStamp("CreationTimeStamp");
                timeStamp.AddChildNode("Version", 1000);
                timeStamp.AddChildNode("Year", 2022);
                timeStamp.AddChildNode("Month", 01);
                timeStamp.AddChildNode("Day", 01);
                timeStamp.AddChildNode("Hour", 0);
                timeStamp.AddChildNode("Minute", 0);
                timeStamp.AddChildNode("Second", 0);
                timeStamp.AddChildNode("Millisecond", 0);

                return timeStamp;
            }

            Node GetSceneInfo()
            {
                Node sceneInfo("SceneInfo");
                sceneInfo.AddProperty("SceneInfo::GlobalInfo");
                sceneInfo.AddProperty("UserData");
                sceneInfo.AddChildNode("Type", "UserData");
                sceneInfo.AddChildNode("Version", 100);
                sceneInfo.AddChildNode(GetMetaData());

                return sceneInfo;
            }

            Node GetMetaData()
            {
                Node metaData("MetaData");
                metaData.AddChildNode("Version", 100);
                metaData.AddChildNode("Title", "");
                metaData.AddChildNode("Subject", "");
                metaData.AddChildNode("Author", "");
                metaData.AddChildNode("Keywords", "");
                metaData.AddChildNode("Revision", "");
                metaData.AddChildNode("Comment", "");

                return metaData;
            }

        private:
            std::vector<Node> basicNodes;
    };

} // namespace ROS2