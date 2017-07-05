/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <ModuleConfiguration.hpp>
#include <core/mw/CoreModule.hpp>

#if CORE_IS_BOOTLOADER_BRIDGE
#include <core/mw/BootMsg.hpp>
#endif

class Module:
    public core::mw::CoreModule
{
public:
// --- DEVICES ----------------------------------------------------------------
// ----------------------------------------------------------------------------

    static bool
    initialize();


#if CORE_IS_BOOTLOADER_BRIDGE
    void
    setBootloaderMasterType(
        core::mw::bootloader::MessageType type
    );
#endif
    Module();
    virtual ~Module() {}
};
