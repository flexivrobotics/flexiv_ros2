/**
 * @file rdk_compat.hpp
 * @brief Name tables of the RDK enums, for building against RDK v2.1 as well as v2.2. RDK v2.2 turned
 * the v2.1 inline maps (kModeNames, ...) into functions (ModeNames(), ...).
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__RDK_COMPAT_HPP_
#define FLEXIV_HARDWARE__RDK_COMPAT_HPP_

#include <map>
#include <string>

#include "flexiv/rdk/data.hpp"
#include "flexiv/rdk/mode.hpp"

// global_vars.hpp first ships with RDK v2.2
#if __has_include(<flexiv/rdk/global_vars.hpp>)
#define FLEXIV_HARDWARE_RDK_V2_2 1
#else
#define FLEXIV_HARDWARE_RDK_V2_2 0
#endif

namespace flexiv_hardware::rdk_compat {

inline const std::map<flexiv::rdk::Mode, std::string>& ModeNames()
{
#if FLEXIV_HARDWARE_RDK_V2_2
    return flexiv::rdk::ModeNames();
#else
    return flexiv::rdk::kModeNames;
#endif
}

inline const std::map<flexiv::rdk::JointGroup, std::string>& JointGroupNames()
{
#if FLEXIV_HARDWARE_RDK_V2_2
    return flexiv::rdk::JointGroupNames();
#else
    return flexiv::rdk::kJointGroupNames;
#endif
}

inline const std::map<flexiv::rdk::OperationalStatus, std::string>& OperationalStatusNames()
{
#if FLEXIV_HARDWARE_RDK_V2_2
    return flexiv::rdk::OperationalStatusNames();
#else
    return flexiv::rdk::kOpStatusNames;
#endif
}

} /* namespace flexiv_hardware::rdk_compat */

#endif /* FLEXIV_HARDWARE__RDK_COMPAT_HPP_ */
