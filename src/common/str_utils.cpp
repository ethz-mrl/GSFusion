/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/common/str_utils.hpp"

#include <algorithm>

#include "se/common/filesystem.hpp"

// POSIX systems must have the unistd.h header.
#if __has_include(<unistd.h>)
#    include <wordexp.h>
#endif


namespace se {
namespace str_utils {


bool begins_with(const std::string& s, const std::string& prefix)
{
    if (s.size() >= prefix.size()) {
        return (s.compare(0, prefix.length(), prefix) == 0);
    }
    else {
        return false;
    }
}


bool ends_with(const std::string& s, const std::string& suffix)
{
    if (s.size() >= suffix.size()) {
        return (s.compare(s.length() - suffix.length(), suffix.length(), suffix) == 0);
    }
    else {
        return false;
    }
}


bool is_int(const std::string& s, const bool accept_negative)
{
    // Try to parse the string as a int.
    try {
        size_t int_len = 0;
        const int i = std::stoi(s, &int_len, 0);
        return ((int_len == s.size()) && (accept_negative || (i >= 0)));
    }
    catch (const std::exception& e) {
        return false;
    }
}


bool is_float(const std::string& s, const bool accept_negative)
{
    // Try to parse the string as a float.
    try {
        size_t int_len = 0;
        const float f = std::stof(s, &int_len);
        return ((int_len == s.size()) && (accept_negative || (f >= 0.0f)));
    }
    catch (const std::exception& e) {
        return false;
    }
}


void remove_prefix(std::string& s, const std::string& prefix)
{
    if (begins_with(s, prefix)) {
        s.erase(0, prefix.size());
    }
}


void remove_suffix(std::string& s, const std::string& suffix)
{
    if (ends_with(s, suffix)) {
        s.erase(s.size() - suffix.size());
    }
}


std::vector<std::string> split_str(const std::string& s, const char delim, const bool ignore_consec)
{
    std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        // Empty items result from consecutive occurences of the delimiter.
        if (!ignore_consec || (ignore_consec && (item.size() > 0))) {
            elems.push_back(item);
        }
    }
    return elems;
}


void to_lower(std::string& s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
}


void to_upper(std::string& s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::toupper(c); });
}


std::string bool_to_pretty_str(const bool state, const std::string& state_name, const int width)
{
    std::string l_side = (state_name != "") ? state_name + ":" : "";
    int padding = (width - 1) - state_name.length();
    if (padding > 0) {
        l_side.append(padding, ' ');
    }

    return l_side + (state ? "true" : "false");
}


std::string str_to_pretty_str(const std::string& string, const std::string& string_name, const int width)
{
    std::string l_side = (string_name != "") ? string_name + ":" : "";
    int padding = (width - 1) - string_name.length();
    if (padding > 0) {
        l_side.append(padding, ' ');
    }

    return l_side + string;
}


std::string header_to_pretty_str(const std::string& header_name, const int width)
{
    int l_width_1;
    float frac = (float) width / header_name.size();
    if (frac > 2) {
        l_width_1 = round((float) width / 4);
    }
    else if (frac > 1) {
        l_width_1 = (float) (width - header_name.size()) / 2 - 1;
    }
    else {
        return header_name;
    }
    int l_width_2 = round((float) (width - header_name.size()) / 2) - l_width_1;
    int r_width_1 = l_width_1;
    int r_width_2 = width - l_width_1 - r_width_1 - l_width_2 - header_name.size();

    std::string l_side;
    l_side.append(l_width_1, '=');
    l_side.append(l_width_2, ' ');

    std::string r_side;
    r_side.append(r_width_2, ' ');
    r_side.append(r_width_1, '=');

    return l_side + header_name + r_side;
}


std::string expand_user(const std::string& path)
{
    // Return the path unchanged on errors or non-POSIX systems.
    std::string expanded_path(path);

    // POSIX systems must have the unistd.h header.
#if __has_include(<unistd.h>)
    wordexp_t expansion;
    if (wordexp(path.c_str(), &expansion, WRDE_NOCMD) == 0) {
        if (expansion.we_wordc >= 1) {
            expanded_path = expansion.we_wordv[0];
        }
    }
    wordfree(&expansion);
#endif
    return expanded_path;
}

std::string resolve_relative_path(const std::string& relative_path, const std::string& base_dir)
{
    const stdfs::path relative_path_p(relative_path);
    if (relative_path_p.is_relative()) {
        return base_dir / relative_path_p;
    }
    return relative_path;
}

} // namespace str_utils
} // namespace se
