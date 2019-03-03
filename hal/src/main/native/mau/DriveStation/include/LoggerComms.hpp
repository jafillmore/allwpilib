#pragma once

namespace mau {
    namespace LoggerComms {
        void start();
        void stop();
        void enqueueErrorMessage(uint16_t num_occur, int32_t errorCode, uint8_t flags, const char *details, const char *location, const char *callStack);
    }
}
