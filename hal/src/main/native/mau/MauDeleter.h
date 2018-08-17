#ifndef NATIVE_MAUDELETER_H

#include <VMXPi.h>
#include "MauMap.h"
#include "Translator/include/MauEnumConverter.h"

#define NATIVE_MAUDELETER_H

namespace mau {
    class Deleter {
        VMXPi* pi;
        Mau_ChannelMap* map;
        Mau_EnumConverter* enums;
    public:
        ~Deleter();
        void addVMXPi(VMXPi* newPi);
        void addMauMap(Mau_ChannelMap* newMap);
        void addMauEnums(Mau_EnumConverter* newEnums);
    };

    Deleter::~Deleter() {
        delete pi;
        delete map;
        delete enums;
    }

    void Deleter::addVMXPi(VMXPi* newPi) {
        pi = newPi;
    }

    void Deleter::addMauMap(Mau_ChannelMap* newMap) {
        map = newMap;
    }

    void Deleter::addMauEnums(Mau_EnumConverter* newEnums) {
        enums = newEnums;
    }


}

#endif //NATIVE_MAUDELETER_H
