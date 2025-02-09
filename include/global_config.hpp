/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2025-02-09 00:04:13
 * @LastEditTime: 2025-02-11 22:36:34
 * @FilePath: /toy-example-of-iLQR/include/global_config.hpp
 * Copyright 2025 puyu, All Rights Reserved.
 */

#pragma once
#ifndef __GLOBAL_CONFIG_HPP
#define __GLOBAL_CONFIG_HPP

#include <any>
#include <vector>
#include <unordered_map>

class GlobalConfig {
private:
    static GlobalConfig* instance;
    std::unordered_map<std::string, std::any> config_map;

    GlobalConfig() {}

    GlobalConfig(const GlobalConfig&) = delete;
    GlobalConfig& operator=(const GlobalConfig&) = delete;

    void load_file(const std::string& filePath);

public:
    static GlobalConfig* get_instance(const std::string& filePath = "");
    bool has_key(std::string key_str);

    template <typename T>
    T get_config(const std::string& key) const;

    static void destroy_instance();
};


#endif
