/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/Json/JsonUtils.h>
#include <AzCore/std/sort.h>
#include <AzToolsFramework/Prefab/PrefabDomUtils.h>
#include <Prefab/PrefabTestDataUtils.h>
#include <Prefab/PrefabTestDomUtils.h>
#include <Prefab/PrefabTestFixture.h>

namespace UnitTest
{
    //! Covers the alternate file formats a gem can register with the Prefab Loader.
    class PrefabTemplateFileHandlerTest : public PrefabTestFixture
    {
    protected:
        //! Stands in for an alternate format: JSON behind a marker prefix, so a test can tell whether the
        //! registered handler ran or the file fell through to built-in JSON parsing.
        static constexpr AZStd::string_view AlternateExtension = ".testprefab";
        static constexpr AZStd::string_view AlternateMarker = "ALTERNATE:";

        //! Records what the save handler was asked to write.
        struct SaveRecord
        {
            bool m_wasCalled = false;
            AZ::IO::Path m_path;
            AZStd::string m_content;
        };

        static AZStd::string ToAlternateFormat(AZStd::string_view json)
        {
            return AZStd::string::format("%.*s%.*s", AZ_STRING_ARG(AlternateMarker), AZ_STRING_ARG(json));
        }

        PrefabTemplateLoadFileHandler MakeLoadHandler()
        {
            return [this](AZStd::string_view fileContent, AZ::IO::PathView filePath)
                -> AZ::Outcome<PrefabDom, AZStd::string>
            {
                ++m_loadHandlerCallCount;
                if (!fileContent.starts_with(AlternateMarker))
                {
                    return AZ::Failure(
                        AZStd::string::format("'%.*s' is not in the alternate format.", AZ_STRING_ARG(filePath.Native())));
                }
                return AZ::JsonSerializationUtils::ReadJsonString(fileContent.substr(AlternateMarker.size()));
            };
        }

        PrefabTemplateSaveFileHandler MakeSaveHandler(SaveRecord& record)
        {
            return [&record](const PrefabDom& dom, AZ::IO::PathView absolutePath) -> AZ::Outcome<void, AZStd::string>
            {
                record.m_wasCalled = true;
                record.m_path = absolutePath;
                record.m_content = ToAlternateFormat(PrefabTestDomUtils::DomToString(dom));
                return AZ::Success();
            };
        }

        //! Registers the format and returns a valid empty Prefab serialized in it.
        AZStd::string RegisterAlternateFormat(SaveRecord& record)
        {
            EXPECT_TRUE(m_prefabLoaderInterface->RegisterTemplateFileHandler(
                AlternateExtension, MakeLoadHandler(), MakeSaveHandler(record)));

            PrefabDom emptyPrefabDom = PrefabTestDomUtils::CreatePrefabDom();
            return ToAlternateFormat(PrefabTestDomUtils::DomToString(emptyPrefabDom));
        }

        int m_loadHandlerCallCount = 0;
    };

    TEST_F(PrefabTemplateFileHandlerTest, LoadTemplateFromString_MatchingExtension_UsesRegisteredLoadHandler)
    {
        SaveRecord saveRecord;
        const AZStd::string alternateContent = RegisterAlternateFormat(saveRecord);

        TemplateData templateData;
        templateData.m_filePath = "path/to/prefab.testprefab";
        templateData.m_id = m_prefabLoaderInterface->LoadTemplateFromString(alternateContent, templateData.m_filePath);

        EXPECT_EQ(m_loadHandlerCallCount, 1);
        PrefabTestDataUtils::ValidateTemplateLoad(templateData);
    }

    TEST_F(PrefabTemplateFileHandlerTest, LoadTemplateFromString_MatchingExtensionDifferentCase_UsesRegisteredLoadHandler)
    {
        SaveRecord saveRecord;
        const AZStd::string alternateContent = RegisterAlternateFormat(saveRecord);

        TemplateData templateData;
        templateData.m_filePath = "path/to/prefab.TestPreFab";
        templateData.m_id = m_prefabLoaderInterface->LoadTemplateFromString(alternateContent, templateData.m_filePath);

        EXPECT_EQ(m_loadHandlerCallCount, 1);
        PrefabTestDataUtils::ValidateTemplateLoad(templateData);
    }

    TEST_F(PrefabTemplateFileHandlerTest, LoadTemplateFromString_NonMatchingExtension_FallsBackToJson)
    {
        SaveRecord saveRecord;
        RegisterAlternateFormat(saveRecord);

        TemplateData templateData;
        templateData.m_filePath = "path/to/prefab.prefab";
        templateData.m_id = m_prefabLoaderInterface->LoadTemplateFromString(
            PrefabTestDomUtils::DomToString(PrefabTestDomUtils::CreatePrefabDom()), templateData.m_filePath);

        EXPECT_EQ(m_loadHandlerCallCount, 0);
        PrefabTestDataUtils::ValidateTemplateLoad(templateData);
    }

    TEST_F(PrefabTemplateFileHandlerTest, LoadTemplateFromString_ExtensionOnlyPartOfName_FallsBackToJson)
    {
        SaveRecord saveRecord;
        RegisterAlternateFormat(saveRecord);

        // ".testprefab" appears in the name but is not the extension.
        TemplateData templateData;
        templateData.m_filePath = "path/to/prefab.testprefab.data.prefab";
        templateData.m_id = m_prefabLoaderInterface->LoadTemplateFromString(
            PrefabTestDomUtils::DomToString(PrefabTestDomUtils::CreatePrefabDom()), templateData.m_filePath);

        EXPECT_EQ(m_loadHandlerCallCount, 0);
        PrefabTestDataUtils::ValidateTemplateLoad(templateData);
    }

    TEST_F(PrefabTemplateFileHandlerTest, LoadTemplateFromString_NameIsOnlyTheExtension_FallsBackToJson)
    {
        SaveRecord saveRecord;
        RegisterAlternateFormat(saveRecord);

        // A leading dot is part of the stem, so this file has no extension at all.
        TemplateData templateData;
        templateData.m_filePath = "path/to/.testprefab";
        templateData.m_id = m_prefabLoaderInterface->LoadTemplateFromString(
            PrefabTestDomUtils::DomToString(PrefabTestDomUtils::CreatePrefabDom()), templateData.m_filePath);

        EXPECT_EQ(m_loadHandlerCallCount, 0);
        PrefabTestDataUtils::ValidateTemplateLoad(templateData);
    }

    TEST_F(PrefabTemplateFileHandlerTest, LoadTemplateFromString_HandlerFails_InvalidTemplateIdReturned)
    {
        SaveRecord saveRecord;
        RegisterAlternateFormat(saveRecord);

        // Valid JSON, but missing the marker the format requires.
        const AZStd::string jsonContent = PrefabTestDomUtils::DomToString(PrefabTestDomUtils::CreatePrefabDom());

        AZ_TEST_START_TRACE_SUPPRESSION;
        const TemplateId templateId = m_prefabLoaderInterface->LoadTemplateFromString(jsonContent, "path/to/prefab.testprefab");
        AZ_TEST_STOP_TRACE_SUPPRESSION(1);

        EXPECT_EQ(m_loadHandlerCallCount, 1);
        EXPECT_EQ(templateId, AzToolsFramework::Prefab::InvalidTemplateId);
    }

    TEST_F(PrefabTemplateFileHandlerTest, SaveTemplate_MatchingExtension_UsesRegisteredSaveHandler)
    {
        SaveRecord saveRecord;
        const AZStd::string alternateContent = RegisterAlternateFormat(saveRecord);

        const AZ::IO::Path templatePath = "path/to/prefab.testprefab";
        const TemplateId templateId = m_prefabLoaderInterface->LoadTemplateFromString(alternateContent, templatePath);
        ASSERT_NE(templateId, AzToolsFramework::Prefab::InvalidTemplateId);

        EXPECT_TRUE(m_prefabLoaderInterface->SaveTemplate(templateId));

        EXPECT_TRUE(saveRecord.m_wasCalled);
        EXPECT_EQ(AZStd::string(saveRecord.m_path.Filename().Native()), AZStd::string(templatePath.Filename().Native()));
        // Imported from an alternate format, written back in the same format.
        EXPECT_TRUE(saveRecord.m_content.starts_with(AlternateMarker));
    }

    TEST_F(PrefabTemplateFileHandlerTest, UnregisterTemplateFileHandler_RegisteredExtension_FormatStopsBeingUsed)
    {
        SaveRecord saveRecord;
        const AZStd::string alternateContent = RegisterAlternateFormat(saveRecord);

        EXPECT_TRUE(m_prefabLoaderInterface->UnregisterTemplateFileHandler(AlternateExtension));
        EXPECT_FALSE(m_prefabLoaderInterface->UnregisterTemplateFileHandler(AlternateExtension));

        AZ_TEST_START_TRACE_SUPPRESSION;
        const TemplateId templateId = m_prefabLoaderInterface->LoadTemplateFromString(alternateContent, "path/to/prefab.testprefab");
        AZ_TEST_STOP_TRACE_SUPPRESSION(1);

        EXPECT_EQ(m_loadHandlerCallCount, 0);
        EXPECT_EQ(templateId, AzToolsFramework::Prefab::InvalidTemplateId);
    }

    TEST_F(PrefabTemplateFileHandlerTest, UnregisterTemplateFileHandler_DifferentCase_FormatStopsBeingUsed)
    {
        SaveRecord saveRecord;
        RegisterAlternateFormat(saveRecord);

        EXPECT_TRUE(m_prefabLoaderInterface->UnregisterTemplateFileHandler(".TESTPREFAB"));
        EXPECT_TRUE(m_prefabLoaderInterface->GetRegisteredTemplateFileExtensions().empty());
    }

    TEST_F(PrefabTemplateFileHandlerTest, RegisterTemplateFileHandler_InvalidArguments_Rejected)
    {
        SaveRecord saveRecord;

        AZ_TEST_START_TRACE_SUPPRESSION;
        // No leading dot.
        EXPECT_FALSE(m_prefabLoaderInterface->RegisterTemplateFileHandler(
            "prefab.test", MakeLoadHandler(), MakeSaveHandler(saveRecord)));
        // Nothing after the dot.
        EXPECT_FALSE(m_prefabLoaderInterface->RegisterTemplateFileHandler(".", MakeLoadHandler(), MakeSaveHandler(saveRecord)));
        // Empty.
        EXPECT_FALSE(m_prefabLoaderInterface->RegisterTemplateFileHandler("", MakeLoadHandler(), MakeSaveHandler(saveRecord)));
        // Path separators.
        EXPECT_FALSE(m_prefabLoaderInterface->RegisterTemplateFileHandler(
            "./test", MakeLoadHandler(), MakeSaveHandler(saveRecord)));
        // More than one dot; PathView::Extension() could never return this.
        EXPECT_FALSE(m_prefabLoaderInterface->RegisterTemplateFileHandler(
            ".prefab.test", MakeLoadHandler(), MakeSaveHandler(saveRecord)));
        // Reserved.
        EXPECT_FALSE(m_prefabLoaderInterface->RegisterTemplateFileHandler(
            ".prefab", MakeLoadHandler(), MakeSaveHandler(saveRecord)));
        EXPECT_FALSE(m_prefabLoaderInterface->RegisterTemplateFileHandler(
            ".PREFAB", MakeLoadHandler(), MakeSaveHandler(saveRecord)));
        // Both handlers are required.
        EXPECT_FALSE(m_prefabLoaderInterface->RegisterTemplateFileHandler(
            AlternateExtension, nullptr, MakeSaveHandler(saveRecord)));
        EXPECT_FALSE(m_prefabLoaderInterface->RegisterTemplateFileHandler(AlternateExtension, MakeLoadHandler(), nullptr));
        AZ_TEST_STOP_TRACE_SUPPRESSION(9);

        EXPECT_TRUE(m_prefabLoaderInterface->GetRegisteredTemplateFileExtensions().empty());
    }

    TEST_F(PrefabTemplateFileHandlerTest, GetRegisteredTemplateFileExtensions_ReturnsLowerCasedExtensions)
    {
        SaveRecord saveRecord;
        EXPECT_TRUE(m_prefabLoaderInterface->RegisterTemplateFileHandler(
            ".ZZZ", MakeLoadHandler(), MakeSaveHandler(saveRecord)));
        EXPECT_TRUE(m_prefabLoaderInterface->RegisterTemplateFileHandler(
            ".Aaa", MakeLoadHandler(), MakeSaveHandler(saveRecord)));

        AZStd::vector<AZStd::string> extensions = m_prefabLoaderInterface->GetRegisteredTemplateFileExtensions();
        AZStd::sort(extensions.begin(), extensions.end());

        const AZStd::vector<AZStd::string> expectedExtensions{ ".aaa", ".zzz" };
        EXPECT_EQ(extensions, expectedExtensions);
    }

    TEST_F(PrefabTemplateFileHandlerTest, RegisterTemplateFileHandler_SameExtensionTwice_ReplacesPreviousFormat)
    {
        SaveRecord firstSaveRecord;
        const AZStd::string alternateContent = RegisterAlternateFormat(firstSaveRecord);

        int replacementLoadHandlerCallCount = 0;
        SaveRecord replacementSaveRecord;
        EXPECT_TRUE(m_prefabLoaderInterface->RegisterTemplateFileHandler(
            AlternateExtension,
            [&replacementLoadHandlerCallCount](AZStd::string_view fileContent, AZ::IO::PathView)
                -> AZ::Outcome<PrefabDom, AZStd::string>
            {
                ++replacementLoadHandlerCallCount;
                return AZ::JsonSerializationUtils::ReadJsonString(fileContent.substr(AlternateMarker.size()));
            },
            MakeSaveHandler(replacementSaveRecord)));

        TemplateData templateData;
        templateData.m_filePath = "path/to/prefab.testprefab";
        templateData.m_id = m_prefabLoaderInterface->LoadTemplateFromString(alternateContent, templateData.m_filePath);

        EXPECT_EQ(m_loadHandlerCallCount, 0);
        EXPECT_EQ(replacementLoadHandlerCallCount, 1);
        PrefabTestDataUtils::ValidateTemplateLoad(templateData);
    }
} // namespace UnitTest
