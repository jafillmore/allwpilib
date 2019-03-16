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
    extern AHRS* vmxIMU;
    extern VMXIO* vmxIO;
    extern VMXCAN* vmxCAN;
    extern VMXPower* vmxPower;
    extern VMXThread* vmxThread;

    extern VMXErrorCode* vmxError;

    extern Mau_ChannelMap* channelMap;
    extern Mau_EnumConverter* enumConverter;

    inline Mau_Channel *GetChannel(hal::HAL_HandleEnum hal_handle_type, int channel_index, int32_t *status) {
     	Mau_Channel* mauChannel = channelMap->getChannel(enumConverter->getHandleLabel(hal_handle_type), channel_index);
    	if (mauChannel == nullptr) {
    		*status = MAU_CHANNEL_MAP_ERROR;
    	}
        return mauChannel;
    }
    inline VMXChannelInfo GetChannelInfo(hal::HAL_HandleEnum hal_handle_type, int channel_index) {
     	return channelMap->getChannelInfo(enumConverter->getHandleLabel(hal_handle_type), channel_index);
    }
}

