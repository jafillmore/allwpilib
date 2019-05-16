/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/DriverStation.h"
#include "hal/Power.h"
#include "hal/CAN.h"

#include <thread>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <wpi/mutex.h>
#include <wpi/condition_variable.h>
#include <wpi/mutex.h>
#include <wpi/raw_ostream.h>

#include "HALInitializer.h"
#include "DriveStation/include/socket.hpp"
#include "DriveStation/include/DriverComms.hpp"
#include "DriveStation/include/MauDriveData.h"
#include "MauInternal.h"

static wpi::mutex msgMutex;
static wpi::mutex* mauDataMutex;
static wpi::condition_variable* mauDataSignal;

namespace hal {
    namespace init {
        void InitializeDriverStation(void (*shutdown_handler)(int)) {
            wpi::mutex* newMutex = new wpi::mutex();
            mauDataMutex = newMutex;
            mauDataSignal = Mau_DriveData::getDataSignal();
            mau::comms::setShutdownHandler(shutdown_handler);
        }
    }
}

using namespace hal;

extern "C" {
    void HAL_InitializeDriverStation() {
        mau::comms::start();
    }

    void HAL_ReleaseDSMutex() {

    }

    HAL_Bool HAL_IsNewControlData(void) {
    	  // There is a rollover error condition here. At Packet# = n * (uintmax), this
    	  // will return false when instead it should return true. However, this at a
    	  // 20ms rate occurs once every 2.7 years of DS connected runtime, so not
    	  // worth the cycles to check.
    	  thread_local uint32_t lastCount{0};
    	  uint32_t currentCount = Mau_DriveData::getNewDSDataAvailableCounter();
    	  if (lastCount == currentCount) return false;
    	  lastCount = currentCount;
    	  return true;
    }

    void HAL_WaitForDSData() {
        HAL_WaitForDSDataTimeout(0);
    }

    HAL_Bool HAL_WaitForDSDataTimeout(double timeout) {

	uint32_t prevCount = Mau_DriveData::getNewDSDataAvailableCounter();
	HAL_ControlWord prevControlWord;
	HAL_GetControlWord(&prevControlWord);

        // Update cached input voltage immediately.
        int32_t status = 0;
        mau::comms::setInputVoltage(HAL_GetVinVoltage(&status));
        // Update can bus status immediately.
        float percentBusUtilization;
        uint32_t busOffCount;
        uint32_t txFullCount;
        uint32_t rxErrorCount;
        uint32_t txErrorCount;
        HAL_CAN_GetCANStatus(&percentBusUtilization, &busOffCount, &txFullCount, &rxErrorCount, &txErrorCount, &status);
        if (!status) {
        	mau::comms::setCANStatus(percentBusUtilization, busOffCount, txFullCount, rxErrorCount, txErrorCount);
        }

        std::unique_lock<wpi::mutex> dataLock(*mauDataMutex);
        std::atomic<bool> expired{false};
        auto waitTime = std::chrono::milliseconds((int)timeout);
        if (timeout <= 0) {
            mauDataSignal->wait(dataLock);
        } else {
            expired = (bool)mauDataSignal->wait_for(dataLock, waitTime);
            if(expired) {
                return false;
            }
        }

        mauDataMutex->unlock();

	/* Condition IO Watchdog based upon current enable bit in control word. */
	uint32_t newCount = Mau_DriveData::getNewDSDataAvailableCounter();
	if (newCount != prevCount) {
		HAL_ControlWord newControlWord;
		VMXErrorCode vmxerr;
		HAL_GetControlWord(&newControlWord);
		if (newControlWord.enabled) {
			// Feed Watchdog whenever new control words are received and enabled.
			mau::vmxIO->FeedWatchdog(&vmxerr);
		} else if (prevControlWord.enabled) {
			// Immediately Expire the Watchdog when disabled state is entered.
			mau::vmxIO->ExpireWatchdogNow(&vmxerr);
		}
	}

        return true;
    }

    int32_t HAL_SendError(HAL_Bool isError, int32_t errorCode, HAL_Bool isLVCode,
                          const char* details, const char* location, const char* callStack, HAL_Bool printMsg) {
        // Avoid flooding console by keeping track of previous 5 error
        // messages and only printing again if they're longer than 1 second old.
        static constexpr int KEEP_MSGS = 5;
        std::lock_guard<wpi::mutex> lock(msgMutex);
        static std::string prevMsg[KEEP_MSGS];
        static std::chrono::time_point<std::chrono::steady_clock>
                prevMsgTime[KEEP_MSGS];
        static bool initialized = false;
        if (!initialized) {
            for (int i = 0; i < KEEP_MSGS; i++) {
                prevMsgTime[i] =
                        std::chrono::steady_clock::now() - std::chrono::seconds(2);
            }
            initialized = true;
        }

        auto curTime = std::chrono::steady_clock::now();
        int i;
        for (i = 0; i < KEEP_MSGS; ++i) {
            if (prevMsg[i] == details) break;
        }
        int retval = 0;
        if (i == KEEP_MSGS || (curTime - prevMsgTime[i]) >= std::chrono::seconds(1)) {

        uint16_t num_occur = 1;
#if 0 /* Previous implemention */

        	printMsg = true;
            if (printMsg) {
                if (location && location[0] != '\0') {
                    std::fprintf(stderr, "%s at %s: ", isError ? "Error" : "Warning",
                                 location);
                }
                std::fprintf(stderr, "%s\n", details);
                if (callStack && callStack[0] != '\0') {
                    std::fprintf(stderr, "%s\n", callStack);
                }
                 mau::comms::enqueueErrorMessage(num_occur, errorCode, isError ? 1 : 0, details, location, callStack);
            }
#endif

            wpi::StringRef detailsRef{details};
            wpi::StringRef locationRef{location};
            wpi::StringRef callStackRef{callStack};

            // 1 tag, 4 timestamp, 2 seqnum
            // 2 numOccur, 4 error code, 1 flags, 6 strlen
            // 1 extra needed for padding on Netcomm end.
            size_t baseLength = 21;

            if (baseLength + detailsRef.size() + locationRef.size() +
                    callStackRef.size() <=
                65536) {
                // Pass through
                retval = mau::comms::enqueueErrorMessage(num_occur, errorCode, isError ? 1 : 0, details, location, callStack);
            } else if (baseLength + detailsRef.size() > 65536) {
              // Details too long, cut both location and stack
              auto newLen = 65536 - baseLength;
              std::string newDetails{details, newLen};
              char empty = '\0';
              retval = mau::comms::enqueueErrorMessage(num_occur, errorCode, isError ? 1 : 0, newDetails.c_str(), &empty, &empty);
            } else if (baseLength + detailsRef.size() + locationRef.size() > 65536) {
              // Location too long, cut stack
              auto newLen = 65536 - baseLength - detailsRef.size();
              std::string newLocation{location, newLen};
              char empty = '\0';
              retval = mau::comms::enqueueErrorMessage(num_occur, errorCode, isError ? 1 : 0, details, newLocation.c_str(), &empty);
            } else {
              // Stack too long
              auto newLen = 65536 - baseLength - detailsRef.size() - locationRef.size();
              std::string newCallStack{callStack, newLen};
              retval = mau::comms::enqueueErrorMessage(num_occur, errorCode, isError ? 1 : 0, details, location, newCallStack.c_str());
            }
            if (printMsg) {
              if (location && location[0] != '\0') {
                wpi::errs() << (isError ? "Error" : "Warning") << " at " << location
                            << ": ";
              }
              wpi::errs() << details << "\n";
              if (callStack && callStack[0] != '\0') {
                wpi::errs() << callStack << "\n";
              }
            }
            if (i == KEEP_MSGS) {
                // replace the oldest one
                i = 0;
                auto first = prevMsgTime[0];
                for (int j = 1; j < KEEP_MSGS; ++j) {
                    if (prevMsgTime[j] < first) {
                        first = prevMsgTime[j];
                        i = j;
                    }
                }
                prevMsg[i] = details;
            }
            prevMsgTime[i] = curTime;
        }
        return retval;
    }

    int32_t HAL_GetControlWord(HAL_ControlWord* controlWord) {
        *controlWord = Mau_DriveData::readControlWord();
        return 0;
    }

    HAL_AllianceStationID HAL_GetAllianceStation(int32_t* status) {
        *status = 0;
        return Mau_DriveData::readAllianceID();
    }

    int32_t HAL_GetJoystickAxes(int32_t joystickNum, HAL_JoystickAxes* axes) {
        Mau_DriveData::scribeJoyAxes(joystickNum, axes);
        return axes->count;
    }

    int32_t HAL_GetJoystickPOVs(int32_t joystickNum, HAL_JoystickPOVs* povs) {
        Mau_DriveData::scribeJoyPOVs(joystickNum, povs);
        return povs->count;
    }

    int32_t HAL_GetJoystickButtons(int32_t joystickNum, HAL_JoystickButtons* buttons) {
        Mau_DriveData::scribeJoyButtons(joystickNum, buttons);
        return 0;
    }

    /**
     * Retrieve the Joystick Descriptor for particular slot
     * @param desc [out] descriptor (data transfer object) to fill in.  desc is
     * filled in regardless of success. In other words, if descriptor is not
     * available, desc is filled in with default values matching the init-values in
     * Java and C++ Driverstation for when caller requests a too-large joystick
     * index.
     *
     * @return error code reported from Network Comm back-end.  Zero is good,
     * nonzero is bad.
     */
    int32_t HAL_GetJoystickDescriptor(int32_t joystickNum, HAL_JoystickDescriptor* desc) {
        Mau_DriveData::scribeJoyDescriptor(joystickNum, desc);
        return 0;
    }

    HAL_Bool HAL_GetJoystickIsXbox(int32_t joystickNum) {
        return Mau_DriveData::readJoyIsXbox(joystickNum);
    }

    int32_t HAL_GetJoystickType(int32_t joystickNum) {
        return Mau_DriveData::readJoyType(joystickNum);
    }

    char* HAL_GetJoystickName(int32_t joystickNum) {
        char* name = new char[250];
        Mau_DriveData::scribeJoyName(joystickNum, name);
        return name;
    }

    void HAL_FreeJoystickName(char* name) {
        std::free(name);
    }

    int32_t HAL_GetJoystickAxisType(int32_t joystickNum, int32_t axis) {
        return Mau_DriveData::readJoyAxisType(joystickNum, axis);
    }

    int32_t HAL_SetJoystickOutputs(int32_t joystickNum, int64_t outputs, int32_t leftRumble, int32_t rightRumble) {
        Mau_DriveData::updateJoyOutputs(joystickNum, outputs, leftRumble, rightRumble);
        return 0;
    }

    double HAL_GetMatchTime(int32_t* status) {
        return Mau_DriveData::readMatchTime();
    }

    int HAL_GetMatchInfo(HAL_MatchInfo* info) {
    	info->eventName[0] = 0;
    	info->gameSpecificMessage[0] = 0;
	info->gameSpecificMessageSize = 0;
        Mau_DriveData::scribeMatchInfo(info);
        return 0;
    }

    void HAL_FreeMatchInfo(HAL_MatchInfo* info) {
    }
}

void HAL_ObserveUserProgramStarting(void) {}
void HAL_ObserveUserProgramDisabled(void) {}
void HAL_ObserveUserProgramAutonomous(void) {}
void HAL_ObserveUserProgramTeleop(void) {}
void HAL_ObserveUserProgramTest(void) {}
