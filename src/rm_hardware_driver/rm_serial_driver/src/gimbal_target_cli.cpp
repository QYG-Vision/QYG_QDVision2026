#include "rm_serial_driver/gimbal_target_cli.hpp"

#include <cmath>
#include <exception>

namespace qd::serial_driver {
namespace {

    std::optional<double> parseFiniteDouble(const std::string& argument) noexcept {
        try {
            size_t parsed_length = 0;
            const double value = std::stod(argument, &parsed_length);
            if (parsed_length != argument.size() || !std::isfinite(value)) {
                return std::nullopt;
            }
            return value;
        } catch (const std::exception&) {
            return std::nullopt;
        }
    }

} // namespace

std::optional<GimbalTargetDegrees>
parseGimbalTargetArguments(const std::vector<std::string>& arguments) noexcept {
    if (arguments.size() != 2U) {
        return std::nullopt;
    }

    const auto yaw = parseFiniteDouble(arguments[0]);
    const auto pitch = parseFiniteDouble(arguments[1]);
    if (!yaw.has_value() || !pitch.has_value()) {
        return std::nullopt;
    }
    return GimbalTargetDegrees { yaw.value(), pitch.value() };
}

} // namespace qd::serial_driver
