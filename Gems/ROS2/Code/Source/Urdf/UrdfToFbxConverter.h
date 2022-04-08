/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <any>
#include <string>

#include "UrdfParser.h"

namespace ROS2
{
    class Fbx
    {
        public:
            using Property = std::any;
            using Properties = std::vector<Property>;

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



        private:
            std::vector<Node> basicNodes;
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //! Class for conversion from URDF to Filmbox (.fbx) files
    class UrdfToFbxConverter
    {
        public:
            std::string ConvertUrdfToFbx(const std::string & urdfString);

        private:

    };

} // namespace ROS2