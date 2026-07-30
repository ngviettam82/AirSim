// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_Ros2TopicName_hpp
#define msr_airlib_Ros2TopicName_hpp

#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>

namespace msr
{
namespace airlib
{

    inline bool isRos2TopicAsciiAlpha(unsigned char ch)
    {
        return (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z');
    }

    inline bool isRos2TopicAsciiAlphaNumeric(unsigned char ch)
    {
        return isRos2TopicAsciiAlpha(ch) || (ch >= '0' && ch <= '9');
    }

    inline uint64_t stableRos2TopicHash(const std::string& value)
    {
        uint64_t hash = 14695981039346656037ULL;
        for (const unsigned char ch : value) {
            hash ^= static_cast<uint64_t>(ch);
            hash *= 1099511628211ULL;
        }
        return hash;
    }

    // Converts an AirSim identifier into one valid ROS 2 topic token. Normal
    // names remain unchanged; changed names receive a stable hash suffix so
    // they stay distinct after normalization.
    inline std::string normalizeRos2TopicToken(const std::string& value, const char* fallback)
    {
        constexpr size_t kMaxTokenLength = 64;
        std::string normalized;
        bool last_was_underscore = false;
        bool changed = false;
        for (const unsigned char ch : value) {
            if (isRos2TopicAsciiAlphaNumeric(ch)) {
                normalized.push_back(static_cast<char>(ch));
                last_was_underscore = false;
            }
            else {
                if (ch != '_' || last_was_underscore)
                    changed = true;
                if (!last_was_underscore) {
                    normalized.push_back('_');
                    last_was_underscore = true;
                }
            }
        }

        while (!normalized.empty() && normalized.back() == '_') {
            normalized.pop_back();
            changed = true;
        }
        if (normalized.empty()) {
            normalized = fallback;
            changed = true;
        }
        if (!isRos2TopicAsciiAlpha(static_cast<unsigned char>(normalized.front()))) {
            normalized = std::string(fallback) +
                (normalized.front() == '_' ? "" : "_") + normalized;
            changed = true;
        }

        if (normalized.size() > kMaxTokenLength)
            changed = true;
        if (changed) {
            std::ostringstream suffix;
            suffix << "_h" << std::hex << stableRos2TopicHash(value);
            const std::string suffix_string = suffix.str();
            const size_t prefix_length = kMaxTokenLength > suffix_string.size()
                ? kMaxTokenLength - suffix_string.size()
                : 1;
            if (normalized.size() > prefix_length)
                normalized.resize(prefix_length);
            normalized += suffix_string;
        }
        return normalized;
    }

}
} // namespace msr::airlib

#endif
