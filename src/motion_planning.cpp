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

std::vector<std::vector<double>> create_lane_border(std::vector<std::vector<double>> ref_line,
                                                    double width) {
    std::vector<std::vector<double>> border(2);
    CubicSpline2D sp = CubicSpline2D(ref_line[0], ref_line[1]);
    for (double s = 0.0; s < sp.s.back(); s += 0.1) {
        Eigen::Vector2d pos = sp.calc_position(s);
        double yaw = sp.calc_yaw(s);
        double lx = pos.x() - width * sin(yaw);
        double ly = pos.y() + width * cos(yaw);
        border[0].emplace_back(lx);
        border[1].emplace_back(ly);
    }

    return border;
}

struct Outlook {
    int rows;
    int cols;
    int colors;
    std::vector<float> data;
};

void imshow(const Outlook& out, std::vector<double> para) {
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
                "support "
                "linestyle");
            imshow_func = nullptr;
        }
    }

    std::vector<double> state_list{0, 0, 0};

    PyObject* vehicle_state = matplotlibcpp::detail::get_array(state_list);
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
        spdlog::info(fmt::format("config parameters:\n{}", YAML::Dump(config)));
    } catch (const YAML::Exception& e) {
        spdlog::error(fmt::format("Error parsing YAML file: {}", e.what()));
        return 1;
    }

    std::vector<double> reference_x =
        config["laneline"]["reference"]["x"].as<std::vector<double>>();
    std::vector<double> reference_y =
        config["laneline"]["reference"]["y"].as<std::vector<double>>();
    std::vector<double> border = config["laneline"]["border"].as<std::vector<double>>();
    std::vector<double> center_line = config["laneline"]["center_line"].as<std::vector<double>>();

    std::vector<std::vector<double>> traj =
        CubicSpline2D::calc_spline_course(reference_x, reference_y, 0.1);

    std::vector<std::vector<std::vector<double>>> borders;
    std::vector<std::vector<std::vector<double>>> center_lines = {traj};
    for (double w : border) {
        std::vector<std::vector<double>> border = create_lane_border({reference_x, reference_y}, w);
        borders.emplace_back(border);
    }
    for (double w : center_line) {
        std::vector<std::vector<double>> border = create_lane_border({reference_x, reference_y}, w);
        center_lines.emplace_back(border);
    }

    Outlook outlook;
    std::string vehicle_pic_path =
        "/home/puyu/Codes/toy-example-of-iLQR/images/materials/"
        "car_cyan.mat.txt";
    outlook.data = utils::imread(vehicle_pic_path, outlook.rows, outlook.cols, outlook.colors);

    for (size_t i = 0; i < borders.size(); ++i) {
        plt::plot(borders[i][0], borders[i][1], "-k");
    }
    for (size_t i = 0; i < center_lines.size(); ++i) {
        plt::plot(center_lines[i][0], center_lines[i][1], "--k");
    }
    imshow(outlook, {4.5, 2.0});
    plt::xlim(-5, 45);
    plt::ylim(-5, 15);
    plt::show();

    return 0;
}
