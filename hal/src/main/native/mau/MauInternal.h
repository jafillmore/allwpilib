#pragma once

#include <string>
#include <AHRS.h>
#include <VMXIO.h>
#include <VMXCAN.h>
#include <VMXTime.h>
#include <VMXPower.h>
#include <VMXThread.h>
#include <VMXErrors.h>
#include <VMXChannel.h>
#include "MauMap.h"
#include "Translator/include/MauEnumConverter.h"
#include "MauErrors.h"
#include "hal/handles/HandlesInternal.h"

namespace mau {
    extern vmx::AHRS* vmxIMU;
    extern VMXIO* vmxIO;
    extern VMXCAN* vmxCAN;
    extern VMXPower* vmxPower;
    extern VMXThread* vmxThread;

    extern VMXErrorCode* vmxError;

    extern Mau_ChannelMap* channelMap;
    extern Mau_EnumConverter* enumConverter;

    inline VMXChannelInfo GetChannelInfo(hal::HAL_HandleEnum hal_handle_type, int channel_index) {
    	VMXChannelInfo chan_info =
    			channelMap->getChannelInfo(enumConverter->getHandleLabel(hal_handle_type), channel_index);
    	// Update capabilities provided by underlying VMX-pi HAL
    	VMXChannelType chan_type;
    	vmxIO->GetChannelCapabilities(chan_info.index, chan_type, chan_info.capabilities);
    	return chan_info;
    }
}

