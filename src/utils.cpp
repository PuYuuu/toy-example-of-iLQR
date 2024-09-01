#include "utils.hpp"

#include <fmt/core.h>
#include <spdlog/spdlog.h>

#include <fstream>

#include "matplotlibcpp.h"

using std::string;
using namespace Eigen;
namespace plt = matplotlibcpp;

namespace utils {

std::vector<float> imread(std::string filename, int& rows, int& cols, int& colors) {
    std::vector<float> image;
    std::ifstream file(filename);

    if (!file.is_open()) {
        spdlog::error(fmt::format("open {} failed !", filename));
        return image;
    }

    std::string line;
    getline(file, line);
    if (line != "Convert from PNG") {
        spdlog::error(fmt::format("this format is not supported: {}", filename));
        return image;
    }
    getline(file, line);
    std::istringstream iss(line);
    iss >> rows >> cols >> colors;
    image.resize(rows * cols * colors);
    int idx = 0;
    while (getline(file, line)) {
        std::istringstream iss(line);
        for (int i = 0; i < colors; ++i) {
            iss >> image[idx++];
        }
    }
    file.close();

    // directly return will trigger RVO (Return Value Optimization)
    return std::move(image);
}

}  // namespace utils
