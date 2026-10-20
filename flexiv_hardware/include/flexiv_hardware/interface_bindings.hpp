/**
 * @file interface_bindings.hpp
 * @brief Binding of the exported ros2_control interfaces to the driver buffers they mirror.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__INTERFACE_BINDINGS_HPP_
#define FLEXIV_HARDWARE__INTERFACE_BINDINGS_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>

namespace flexiv_hardware {

/**
 * @brief Exported interfaces, each bound to the driver buffer it mirrors. ros2_control owns the
 * interface values, so the buffers are copied in and out once per cycle rather than shared.
 */
class InterfaceBindings
{
public:
    /** @brief Create a state interface mirroring [buffer], starting from its current value. */
    hardware_interface::StateInterface::ConstSharedPtr BindState(
        const std::string& prefix, const std::string& name, const double* buffer)
    {
        auto interface = std::make_shared<hardware_interface::StateInterface>(
            Describe(prefix, name));
        static_cast<void>(interface->set_value(*buffer, true));
        states_.emplace_back(interface, buffer);
        return interface;
    }

    /** @brief Create a command interface mirroring [buffer], starting from its current value. */
    hardware_interface::CommandInterface::SharedPtr BindCommand(
        const std::string& prefix, const std::string& name, double* buffer)
    {
        auto interface = std::make_shared<hardware_interface::CommandInterface>(
            Describe(prefix, name));
        static_cast<void>(interface->set_value(*buffer, true));
        commands_.emplace_back(interface, buffer);
        return interface;
    }

    /**
     * @brief [Non-blocking] Copy every state buffer into its interface. An interface momentarily
     * locked by another thread is updated on the next call.
     */
    void WriteStates() const
    {
        for (const auto& [interface, buffer] : states_) {
            static_cast<void>(interface->set_value(*buffer, false));
        }
    }

    /**
     * @brief [Non-blocking] Copy every command interface into its buffer. A buffer whose interface
     * is momentarily locked by another thread keeps its last value.
     */
    void ReadCommands() const
    {
        for (const auto& [interface, buffer] : commands_) {
            if (const auto value = interface->get_optional()) {
                *buffer = *value;
            }
        }
    }

    /**
     * @brief [Blocking] Copy every command buffer into its interface, so that what the driver set
     * is what the controllers see and what the next ReadCommands() returns.
     */
    void WriteCommands() const
    {
        for (const auto& [interface, buffer] : commands_) {
            static_cast<void>(interface->set_value(*buffer, true));
        }
    }

private:
    static hardware_interface::InterfaceDescription Describe(
        const std::string& prefix, const std::string& name)
    {
        hardware_interface::InterfaceInfo info {};
        info.name = name;
        return hardware_interface::InterfaceDescription(prefix, info);
    }

    std::vector<std::pair<hardware_interface::StateInterface::SharedPtr, const double*>> states_;
    std::vector<std::pair<hardware_interface::CommandInterface::SharedPtr, double*>> commands_;
};

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__INTERFACE_BINDINGS_HPP_ */
