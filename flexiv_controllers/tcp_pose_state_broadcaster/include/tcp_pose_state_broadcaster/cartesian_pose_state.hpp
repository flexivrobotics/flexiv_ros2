/**
 * @file cartesian_pose_state.hpp
 * @brief Sensor interface to read the Cartesian pose. Adapted from
 * ros2_control/controller_interface/include/semantic_components/force_torque_sensor.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef SEMANTIC_COMPONENTS__CARTESIAN_POSE_STATE_HPP_
#define SEMANTIC_COMPONENTS__CARTESIAN_POSE_STATE_HPP_

#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "semantic_components/semantic_component_interface.hpp"

namespace semantic_components {
class CartesianPoseState : public SemanticComponentInterface<geometry_msgs::msg::Pose>
{
public:
    /// Constructor for "standard" 7D Cartesian pose
    explicit CartesianPoseState(const std::string& name)
    : SemanticComponentInterface(name, 7)
    {
        interface_names_.emplace_back(name_ + "/" + "position.x");
        interface_names_.emplace_back(name_ + "/" + "position.y");
        interface_names_.emplace_back(name_ + "/" + "position.z");
        interface_names_.emplace_back(name_ + "/" + "orientation.x");
        interface_names_.emplace_back(name_ + "/" + "orientation.y");
        interface_names_.emplace_back(name_ + "/" + "orientation.z");
        interface_names_.emplace_back(name_ + "/" + "orientation.w");

        // Set all interfaces existing
        std::fill(existing_axes_.begin(), existing_axes_.end(), true);

        // Set default position and orientation values to NaN
        std::fill(positions_.begin(), positions_.end(), std::numeric_limits<double>::quiet_NaN());
        std::fill(
            orientations_.begin(), orientations_.end(), std::numeric_limits<double>::quiet_NaN());
    }

    /// Constructor for "custom" Cartesian pose
    CartesianPoseState(const std::string& interface_position_x,
        const std::string& interface_position_y, const std::string& interface_position_z,
        const std::string& interface_orientation_x, const std::string& interface_orientation_y,
        const std::string& interface_orientation_z, const std::string& interface_orientation_w)
    : SemanticComponentInterface("", 7)
    {
        auto check_and_add_interface = [this](const std::string& interface_name, const int index) {
            if (!interface_name.empty()) {
                interface_names_.emplace_back(interface_name);
                existing_axes_[index] = true;
            } else {
                existing_axes_[index] = false;
            }
        };

        check_and_add_interface(interface_position_x, 0);
        check_and_add_interface(interface_position_y, 1);
        check_and_add_interface(interface_position_z, 2);
        check_and_add_interface(interface_orientation_x, 3);
        check_and_add_interface(interface_orientation_y, 4);
        check_and_add_interface(interface_orientation_z, 5);
        check_and_add_interface(interface_orientation_w, 6);

        // Set default position and orientation values to NaN
        std::fill(positions_.begin(), positions_.end(), std::numeric_limits<double>::quiet_NaN());
        std::fill(
            orientations_.begin(), orientations_.end(), std::numeric_limits<double>::quiet_NaN());
    }

    virtual ~CartesianPoseState() = default;

    /// Return positions
    std::array<double, 3>& get_positions()
    {
        size_t position_interface_counter = 0;
        for (size_t i = 0; i < 3; ++i) {
            if (existing_axes_[i]) {
                positions_[i] = state_interfaces_[position_interface_counter].get().get_value();
                ++position_interface_counter;
            }
        }
        return positions_;
    }

    /// Return orientations
    std::array<double, 4>& get_orientations()
    {
        auto orientation_interface_counter
            = std::count(existing_axes_.begin(), existing_axes_.begin() + 3, true);
        for (size_t i = 3; i < 7; ++i) {
            if (existing_axes_[i]) {
                orientations_[i - 3]
                    = state_interfaces_[orientation_interface_counter].get().get_value();
                ++orientation_interface_counter;
            }
        }
        return orientations_;
    }

    /// Return Pose message with positions and orientations
    bool get_values_as_message(geometry_msgs::msg::Pose& message)
    {
        get_positions();
        get_orientations();

        message.position.x = positions_[0];
        message.position.y = positions_[1];
        message.position.z = positions_[2];
        message.orientation.x = orientations_[0];
        message.orientation.y = orientations_[1];
        message.orientation.z = orientations_[2];
        message.orientation.w = orientations_[3];

        return true;
    }

protected:
    std::array<bool, 7> existing_axes_;
    std::array<double, 3> positions_;
    std::array<double, 4> orientations_;
};

} /* namespace semantic_components */

#endif /* SEMANTIC_COMPONENTS__CARTESIAN_POSE_STATE_HPP_ */
