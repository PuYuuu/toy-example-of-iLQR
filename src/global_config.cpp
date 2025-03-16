/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2025-02-09 00:04:06
 * @LastEditTime: 2025-02-11 22:44:00
 * @FilePath: /toy-example-of-iLQR/src/global_config.cpp
 * Copyright 2025 puyu, All Rights Reserved.
 */

#include "global_config.hpp"

#include <yaml-cpp/yaml.h>

#include <iostream>

GlobalConfig* GlobalConfig::instance = nullptr;

void GlobalConfig::load_file(const std::string& filePath) {
    YAML::Node config;
    try {
        config = YAML::LoadFile(filePath);

        config_map["max_simulation_time"] = config["max_simulation_time"].as<double>();
        config_map["delta_t"] = config["delta_t"].as<double>();

        config_map["lqr/N"] = config["lqr"]["N"].as<int>();
        config_map["lqr/nx"] = config["lqr"]["nx"].as<int>();
        config_map["lqr/nu"] = config["lqr"]["nu"].as<int>();
        config_map["lqr/w_pos"] = config["lqr"]["w_pos"].as<double>();
        config_map["lqr/w_vel"] = config["lqr"]["w_vel"].as<double>();
        config_map["lqr/w_yaw"] = config["lqr"]["w_yaw"].as<double>();
        config_map["lqr/w_acc"] = config["lqr"]["w_acc"].as<double>();
        config_map["lqr/w_stl"] = config["lqr"]["w_stl"].as<double>();
        config_map["lqr/slove_type"] = config["lqr"]["slove_type"].as<std::string>();
        config_map["lqr/alm_rho_init"] = config["lqr"]["alm_rho_init"].as<double>(1.0);
        config_map["lqr/alm_gamma"] = config["lqr"]["alm_gamma"].as<double>(0.0);
        config_map["lqr/max_rho"] = config["lqr"]["max_rho"].as<double>(100.0);
        config_map["lqr/max_mu"] = config["lqr"]["max_mu"].as<double>(1000.0);
        config_map["lqr/obstacle_exp_q1"] = config["lqr"]["obstacle_exp_q1"].as<double>();
        config_map["lqr/obstacle_exp_q2"] = config["lqr"]["obstacle_exp_q2"].as<double>();
        config_map["lqr/state_exp_q1"] = config["lqr"]["state_exp_q1"].as<double>();
        config_map["lqr/state_exp_q2"] = config["lqr"]["state_exp_q2"].as<double>();
        config_map["lqr/use_last_solution"] = config["lqr"]["use_last_solution"].as<bool>();

        config_map["iteration/max_iter"] = config["iteration"]["max_iter"].as<int>();
        config_map["iteration/init_lamb"] = config["iteration"]["init_lamb"].as<double>();
        config_map["iteration/lamb_decay"] = config["iteration"]["lamb_decay"].as<double>();
        config_map["iteration/lamb_amplify"] = config["iteration"]["lamb_amplify"].as<double>();
        config_map["iteration/max_lamb"] = config["iteration"]["max_lamb"].as<double>();
        config_map["iteration/convergence_threshold"] =
            config["iteration"]["convergence_threshold"].as<double>();
        config_map["iteration/accept_step_threshold"] =
            config["iteration"]["accept_step_threshold"].as<double>();

        config_map["vehicle/reference_point"] =
            config["vehicle"]["reference_point"].as<std::string>("gravity_center");
        config_map["vehicle/target_velocity"] = config["vehicle"]["target_velocity"].as<double>();
        config_map["vehicle/wheelbase"] = config["vehicle"]["wheelbase"].as<double>();
        config_map["vehicle/width"] = config["vehicle"]["width"].as<double>();
        config_map["vehicle/length"] = config["vehicle"]["length"].as<double>();
        config_map["vehicle/velo_max"] = config["vehicle"]["velo_max"].as<double>();
        config_map["vehicle/velo_min"] = config["vehicle"]["velo_min"].as<double>();
        config_map["vehicle/yaw_lim"] = config["vehicle"]["yaw_lim"].as<double>();
        config_map["vehicle/acc_max"] = config["vehicle"]["acc_max"].as<double>();
        config_map["vehicle/acc_min"] = config["vehicle"]["acc_min"].as<double>();
        config_map["vehicle/stl_lim"] = config["vehicle"]["stl_lim"].as<double>();
        config_map["vehicle/d_safe"] = config["vehicle"]["d_safe"].as<double>();

        config_map["laneline/reference/x"] =
            config["laneline"]["reference"]["x"].as<std::vector<double>>();
        config_map["laneline/reference/y"] =
            config["laneline"]["reference"]["y"].as<std::vector<double>>();
        config_map["laneline/border"] = config["laneline"]["border"].as<std::vector<double>>();
        config_map["laneline/center_line"] =
            config["laneline"]["center_line"].as<std::vector<double>>();

        config_map["initial_condition"] =
            config["initial_condition"].as<std::vector<std::vector<double>>>();

        config_map["visualization/show_reference_line"] =
            config["visualization"]["show_reference_line"].as<bool>(false);
        config_map["visualization/show_obstacle_boundary"] =
            config["visualization"]["show_obstacle_boundary"].as<bool>(false);
        if (config["visualization"]) {
            if (config["visualization"]["x_lim"]) {
                config_map["visualization/x_lim"] =
                    config["visualization"]["x_lim"].as<std::vector<double>>();
            }
            if (config["visualization"]["y_lim"]) {
                config_map["visualization/y_lim"] =
                    config["visualization"]["y_lim"].as<std::vector<double>>();
            }
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "Error parsing YAML file: " << e.what() << std::endl;
    }
}

bool GlobalConfig::has_key(std::string key_str) {
    return config_map.find(key_str) != config_map.end();
}

GlobalConfig* GlobalConfig::get_instance(const std::string& filePath /* = "" */) {
    if (instance == nullptr) {
        instance = new GlobalConfig();
        if (!filePath.empty()) {
            instance->load_file(filePath);
        } else {
            std::cerr << "The GlobalConfig singleton class is not initialized before use "
                      << std::endl;
            throw std::runtime_error("GlobalConfig is not initialized!");
        }
    }

    return instance;
}

template <typename T>
T GlobalConfig::get_config(const std::string& key) const {
    auto it = config_map.find(key);
    if (it != config_map.end()) {
        try {
            return std::any_cast<T>(it->second);
        } catch (const std::bad_any_cast& e) {
            std::cerr << "Type mismatch for key: " << key << std::endl;
        }
    } else {
        std::cerr << "Configuration key not found: " << key << std::endl;
    }

    return T();
}

void GlobalConfig::destroy_instance() {
    if (instance) {
        delete instance;
        instance = nullptr;
    }
}

template std::vector<double> GlobalConfig::get_config<std::vector<double>>(
    const std::string& key) const;
template std::vector<std::vector<double>>
GlobalConfig::get_config<std::vector<std::vector<double>>>(const std::string& key) const;
template std::string GlobalConfig::get_config<std::string>(const std::string& key) const;
template int GlobalConfig::get_config<int>(const std::string& key) const;
template double GlobalConfig::get_config<double>(const std::string& key) const;
template bool GlobalConfig::get_config<bool>(const std::string& key) const;
