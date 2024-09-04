#include "cilqr_solver.hpp"
#include "cubic_spline.hpp"
#include "matplotlibcpp.h"
#include "utils.hpp"

#include <fmt/core.h>
#include <getopt.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

namespace plt = matplotlibcpp;

struct Outlook {
    int rows;
    int cols;
    int colors;
    std::vector<float> data;
};

void imshow(const Outlook& out, std::vector<double> state, std::vector<double> para) {
    PyObject* imshow_func = nullptr;
    if (imshow_func == nullptr) {
        Py_Initialize();
        std::filesystem::path source_file_path(__FILE__);
        std::filesystem::path project_path = source_file_path.parent_path().parent_path();
        std::string script_path = project_path / "scripts" / "utils";
        PyRun_SimpleString("import sys");
        PyRun_SimpleString(fmt::format("sys.path.append('{}')", script_path).c_str());

        PyObject* py_name = PyUnicode_DecodeFSDefault("imshow");
        PyObject* py_module = PyImport_Import(py_name);
        Py_DECREF(py_name);
        if (py_module != nullptr) {
            imshow_func = PyObject_GetAttrString(py_module, "imshow");
        }
        if (imshow_func == nullptr || !PyCallable_Check(imshow_func)) {
            spdlog::error(
                "py.imshow call failed and the vehicle drawing will only "
                "support linestyle");
            imshow_func = nullptr;
        }
    }

    std::vector<double> state_list{0, 0, 0};

    PyObject* vehicle_state = matplotlibcpp::detail::get_array(state);
    PyObject* vehicle_para = matplotlibcpp::detail::get_array(para);
    npy_intp dims[3] = {out.rows, out.cols, out.colors};

    const float* imptr = &(out.data[0]);

    PyObject* args = PyTuple_New(3);
    PyTuple_SetItem(args, 0, PyArray_SimpleNewFromData(3, dims, NPY_FLOAT, (void*)imptr));
    PyTuple_SetItem(args, 1, vehicle_state);
    PyTuple_SetItem(args, 2, vehicle_para);

    PyObject* ret = PyObject_CallObject(imshow_func, args);

    Py_DECREF(args);
    if (ret) {
        Py_DECREF(ret);
    }
}

int main(int argc, char** argv) {
    YAML::Node config;
    std::filesystem::path config_path = "../config/config.yaml";

    spdlog::info(fmt::format("config path: {}", config_path.string()));
    try {
        config = YAML::LoadFile(config_path.string());
        spdlog::debug(fmt::format("config parameters:\n{}", YAML::Dump(config)));
    } catch (const YAML::Exception& e) {
        spdlog::error(fmt::format("Error parsing YAML file: {}", e.what()));
        return 1;
    }

    double max_simulation_time = config["max_simulation_time"].as<double>();
    double delta_t = config["delta_t"].as<double>();
    std::vector<double> reference_x =
        config["laneline"]["reference"]["x"].as<std::vector<double>>();
    std::vector<double> reference_y =
        config["laneline"]["reference"]["y"].as<std::vector<double>>();
    std::vector<double> border_widths = config["laneline"]["border"].as<std::vector<double>>();
    std::vector<double> center_line_widths =
        config["laneline"]["center_line"].as<std::vector<double>>();
    std::vector<std::vector<double>> initial_conditions =
        config["initial_condition"].as<std::vector<std::vector<double>>>();
    size_t vehicle_num = initial_conditions.size();

    std::vector<ReferenceLine> borders;
    std::vector<ReferenceLine> center_lines;
    for (double w : border_widths) {
        ReferenceLine reference(reference_x, reference_y, w);
        borders.emplace_back(reference);
    }
    for (double w : center_line_widths) {
        ReferenceLine reference(reference_x, reference_y, w);
        center_lines.emplace_back(reference);
    }

    Outlook outlook_ego;
    Outlook outlook_agent;
    std::string vehicle_pic_path_ego =
        "/home/puyu/Codes/toy-example-of-iLQR/images/materials/car_cyan.mat.txt";
    std::string vehicle_pic_path_agent =
        "/home/puyu/Codes/toy-example-of-iLQR/images/materials/car_white.mat.txt";
    outlook_ego.data =
        utils::imread(vehicle_pic_path_ego, outlook_ego.rows, outlook_ego.cols, outlook_ego.colors);
    outlook_agent.data = utils::imread(vehicle_pic_path_agent, outlook_agent.rows,
                                       outlook_agent.cols, outlook_agent.colors);

    std::vector<RoutingLine> routing_lines(vehicle_num);

    for (size_t idx = 0; idx < vehicle_num; ++idx) {
        size_t line_num = 0;
        double start_s = center_lines[line_num].length();
        double min_diff = -1.0;
        for (size_t l = 0; l < center_lines.size(); ++l) {
            for (size_t i = 1; i < center_lines[l].size(); ++i) {
                double last_diff = hypot(center_lines[l].x[i - 1] - initial_conditions[idx][0],
                                         center_lines[l].y[i - 1] - initial_conditions[idx][1]);
                double cur_diff = hypot(center_lines[l].x[i] - initial_conditions[idx][0],
                                        center_lines[l].y[i] - initial_conditions[idx][1]);
                if (cur_diff > last_diff) {
                    if (min_diff < 0 || last_diff < min_diff) {
                        min_diff = last_diff;
                        line_num = l;
                        start_s = center_lines[l].longitude[i - 1];
                    }
                    break;
                }
            }
        }
        spdlog::debug(fmt::format("idx: {}, line_num: {}, start_s: {}", idx, line_num, start_s));
        for (double t = 0.0; t < max_simulation_time; t += delta_t) {
            double cur_s = start_s + t * initial_conditions[idx][2];
            cur_s = std::min(cur_s, center_lines[line_num].longitude.back());
            Eigen::Vector3d pos = center_lines[line_num].calc_position(cur_s);
            routing_lines[idx].x.push_back(pos.x());
            routing_lines[idx].y.push_back(pos.y());
            routing_lines[idx].yaw.push_back(pos.z());
        }
    }

    for (double t = 0.; t < max_simulation_time; t += delta_t) {
        size_t index = t / delta_t;
        plt::cla();
        for (size_t i = 0; i < borders.size(); ++i) {
            plt::plot(borders[i].x, borders[i].y, "-k");
        }
        for (size_t i = 0; i < center_lines.size(); ++i) {
            plt::plot(center_lines[i].x, center_lines[i].y, "--k");
        }

        imshow(outlook_ego,
               {routing_lines[0].x[index], routing_lines[0].y[index], routing_lines[0].yaw[index]},
               {4.5, 2});
        for (size_t idx = 1; idx < vehicle_num; ++idx) {
            imshow(outlook_agent,
                   {routing_lines[idx].x[index], routing_lines[idx].y[index],
                    routing_lines[idx].yaw[index]},
                   {4.5, 2});
        }

        plt::xlim(routing_lines[0].x[index] - 15, routing_lines[0].x[index] + 25);
        plt::ylim(routing_lines[0].y[index] - 5, routing_lines[0].y[index] + 15);
        plt::pause(delta_t);
    }

    plt::show();

    return 0;
}
