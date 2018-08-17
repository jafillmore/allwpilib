#include <VMXPi.h>
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

namespace mau {
    extern AHRS* vmxIMU;
    extern VMXIO* vmxIO;
    extern VMXCAN* vmxCAN;
    extern VMXPower* vmxPower;
    extern VMXThread* vmxThread;

    extern VMXErrorCode* vmxError;

    extern Mau_ChannelMap* channelMap;
    extern Mau_EnumConverter* enumConverter;
}