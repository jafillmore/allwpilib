/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/Notifier.h"

#include <atomic>
#include <cstdlib>  // For std::atexit()
#include <memory>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

#include <wpi/condition_variable.h>
#include <wpi/mutex.h>

#include "HALInitializer.h"
#include "hal/Errors.h"
#include "hal/HAL.h"
#include "hal/handles/UnlimitedHandleResource.h"
#include <VMXPi.h>
#include "MauTime.h"
#include "MauInternal.h"
#include "hal/cpp/fpga_clock.h"

using namespace hal;

static timer_t notifierTimer;

static wpi::mutex notifierMutex;
static timer_t *notifierAlarm = 0;
static uint64_t closestTrigger{UINT64_MAX};

namespace {

struct Notifier {
  uint64_t triggerTime = UINT64_MAX;
  uint64_t triggeredTime = UINT64_MAX;
  bool active = true;
  wpi::mutex mutex;
  wpi::condition_variable cond;
};

}  // namespace

static std::atomic_flag notifierAtexitRegistered{ATOMIC_FLAG_INIT};
static std::atomic_int notifierRefCount{0};

using namespace hal;

class NotifierHandleContainer
    : public UnlimitedHandleResource<HAL_NotifierHandle, Notifier,
                                     HAL_HandleEnum::Notifier> {
 public:
  void Cleanup() {
    ForEach([](HAL_NotifierHandle handle, Notifier* notifier) {
      {
        std::lock_guard<wpi::mutex> lock(notifier->mutex);
        notifier->active = false;
      }
      notifier->cond.notify_all();  // wake up any waiting threads
    });
  }
  virtual ~NotifierHandleContainer() {
	 Cleanup();
  }
};

static NotifierHandleContainer* notifierHandles;

// Linux high-resolution timer callback, invoked via signal

//#define ENABLE_ALARM_DEBUG

static void alarmCallback(int signum) {
  int32_t status = 0;
  uint64_t currentTime = 0;

  //if (signum != SIGALRM) return;
  if (!notifierHandles) return;
  if (!notifierAlarm) return;

#ifdef ENABLE_ALARM_DEBUG
  struct timespec tp;
  char buffer [80];

  uint64_t currFPGATime = HAL_GetFPGATime(&status);
  uint64_t currFPGADelta = currFPGATime - closestTrigger;

  if (notifierAlarm) {
    if (clock_gettime (CLOCK_MONOTONIC, &tp) == -1)
        perror ("clock_gettime");

    sprintf (buffer, "alarmCallback:  FPGATime:  %lld (delta:  %lld); %ld s %ld ns overrun = %d\n", currFPGATime, currFPGADelta, tp.tv_sec,
                        tp.tv_nsec, timer_getoverrun (*notifierAlarm));
    write (STDOUT_FILENO, buffer, strlen (buffer));
  }
#endif

  std::lock_guard<wpi::mutex> lock(notifierMutex);

#ifdef ENABLE_ALARM_DEBUG
        sprintf (buffer, "alarmCallback:  Got notifierMutex.\n");
        write (STDOUT_FILENO, buffer, strlen (buffer));
#endif

  // the hardware disables itself after each alarm
  closestTrigger = UINT64_MAX;

  // process all notifiers
  notifierHandles->ForEach([&](HAL_NotifierHandle handle, Notifier* notifier) {
    if (notifier->triggerTime == UINT64_MAX) return;
    if (currentTime == 0) currentTime = HAL_GetFPGATime(&status);
    std::unique_lock<wpi::mutex> lock(notifier->mutex);
    if (notifier->triggerTime < currentTime) {
      notifier->triggerTime = UINT64_MAX;
      notifier->triggeredTime = currentTime;
      lock.unlock();
      notifier->cond.notify_all();
    } else if (notifier->triggerTime < closestTrigger) {
      closestTrigger = notifier->triggerTime;
    }
  });

  if (notifierAlarm && (closestTrigger != UINT64_MAX)) {
    struct itimerspec new_value, old_value;
    uint64_t delta;
    if (closestTrigger > currentTime) {
    	delta = closestTrigger - currentTime;
    } else {
    	delta = 1000;
    }

    // Configure one-shot timer to expire at next closest trigger time
    new_value.it_value.tv_sec  = delta / 1000000; // NUM_MICROSECS_PER_SEC
    new_value.it_value.tv_nsec = (delta % 1000000) * 1000; // NUM_MICROSECS_PER_SEC, converted to nanoseconds
    new_value.it_interval.tv_sec = 0;
    new_value.it_interval.tv_nsec = 0;

    if (timer_settime (*notifierAlarm, 0, &new_value, &old_value) == -1) {
        perror ("timer_settime in Notifier alarmCallback.");
#ifdef ENABLE_ALARM_DEBUG
        sprintf (buffer, "Error invoking timer_settime, trying to set new ClosestTrigger:  %llu; delta:  %llu; tv_sec:  %lu; tv_nsec:  %lu (currentTime: %llu; closestTrigger:  %llu)\n", delta, new_value.it_value.tv_sec, new_value.it_value.tv_nsec, currentTime, closestTrigger);
        write (STDOUT_FILENO, buffer, strlen (buffer));
#endif
    } else {
#ifdef ENABLE_ALARM_DEBUG
        sprintf (buffer, "alarmCallback set new delta:  %llu; tv_sec:  %lu; tv_nsec:  %lu (currentTime: %llu; closestTrigger:  %llu)\n", delta, new_value.it_value.tv_sec, new_value.it_value.tv_nsec, currentTime, closestTrigger);
        write (STDOUT_FILENO, buffer, strlen (buffer));
#endif
    }
  } else {
#ifdef ENABLE_ALARM_DEBUG
        sprintf (buffer, "alarmCallback No new closest Trigger.\n");
        write (STDOUT_FILENO, buffer, strlen (buffer));
#endif
  }
}

static void cleanupNotifierAtExit() {
  if (notifierAlarm) {
    timer_delete(*notifierAlarm);
    notifierAlarm = nullptr;
  }
}

static void HAL_Mau_NotifierTerminateHandler(void) {
	cleanupNotifierAtExit();
	if (notifierHandles != 0) {
		notifierHandles->Cleanup();
		notifierHandles = 0;
	}
}

namespace hal {
namespace init {
void InitializeNotifier() {
  static NotifierHandleContainer nH;
  notifierHandles = &nH;
  VMXPi *p_vmxpi = VMXPi::getInstance();
  if (p_vmxpi) {
	  p_vmxpi->registerShutdownHandler(HAL_Mau_NotifierTerminateHandler);
  }
}
}  // namespace init
}  // namespace hal

extern "C" {

HAL_NotifierHandle HAL_InitializeNotifier(int32_t* status) {
  hal::init::CheckInit();
  if (!notifierAtexitRegistered.test_and_set())
    std::atexit(cleanupNotifierAtExit);

  if (notifierRefCount.fetch_add(1) == 0) {
    std::lock_guard<wpi::mutex> lock(notifierMutex);
    // create alarm if not already created

    if (!notifierAlarm) {

      struct sigaction action;

      /* SIGALRM for alarm callback */
      memset (&action, 0, sizeof (struct sigaction));
      action.sa_handler = alarmCallback;
      if (sigaction (SIGALRM, &action, NULL) == -1) {
    	  perror ("sigaction");
    	  *status = HAL_HANDLE_ERROR;
    	  return HAL_kInvalidHandle;
      }
      
      if (timer_create (CLOCK_MONOTONIC, NULL, &notifierTimer) == -1) {
    	  perror ("timer_create");
    	  *status = HAL_HANDLE_ERROR;
    	  return HAL_kInvalidHandle;
      } else {
        notifierAlarm = &notifierTimer;
      }
    }
  }

  std::shared_ptr<Notifier> notifier = std::make_shared<Notifier>();
  HAL_NotifierHandle handle = notifierHandles->Allocate(notifier);
  if (handle == HAL_kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
    return HAL_kInvalidHandle;
  }
  return handle;
}

void HAL_StopNotifier(HAL_NotifierHandle notifierHandle, int32_t* status) {
  auto notifier = notifierHandles->Get(notifierHandle);
  if (!notifier) return;

  {
    std::lock_guard<wpi::mutex> lock(notifier->mutex);
    notifier->triggerTime = UINT64_MAX;
    notifier->triggeredTime = 0;
    notifier->active = false;
  }
  notifier->cond.notify_all();  // wake up any waiting threads
}

void HAL_CleanNotifier(HAL_NotifierHandle notifierHandle, int32_t* status) {
  auto notifier = notifierHandles->Free(notifierHandle);
  if (!notifier) return;

  // Just in case HAL_StopNotifier() wasn't called...
  {
    std::lock_guard<wpi::mutex> lock(notifier->mutex);
    notifier->triggerTime = UINT64_MAX;
    notifier->triggeredTime = 0;
    notifier->active = false;
  }
  notifier->cond.notify_all();

  if (notifierRefCount.fetch_sub(1) == 1) {
    // if this was the last notifier, clean up alarm and manager
    // the notifier can call back into our callback, so don't hold the lock
    // here (the atomic fetch_sub will prevent multiple parallel entries
    // into this function)

    // Cleaning up the manager takes up to a second to complete, so don't do
    // that here. Fix it more permanently in 2019...

    // if (notifierAlarm) notifierAlarm->writeEnable(false, status);
    // if (notifierManager) notifierManager->disable(status);

    // std::lock_guard<wpi::mutex> lock(notifierMutex);
    // notifierAlarm = nullptr;
    // notifierManager = nullptr;
    // closestTrigger = UINT64_MAX;
  }
}

void HAL_UpdateNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             uint64_t triggerTime, int32_t* status) {
  auto notifier = notifierHandles->Get(notifierHandle);
  if (!notifier) return;
  if (!notifierAlarm)  return;

#ifdef ENABLE_ALARM_DEBUG
  printf("ERROR:  HAL_UpdateNotifierAlarm entered - new trigger time:  %llu.\n", triggerTime);
  fflush(stdout);
#endif

  {
    std::lock_guard<wpi::mutex> lock(notifier->mutex);
    notifier->triggerTime = triggerTime;
    notifier->triggeredTime = UINT64_MAX;
  }

  std::lock_guard<wpi::mutex> lock(notifierMutex);

  // Update alarm time if closer than current.
  if (triggerTime < closestTrigger) {
    struct itimerspec new_value, old_value;

    bool was_active = (closestTrigger != UINT64_MAX);
    closestTrigger = triggerTime;

    int32_t status = 0;
    uint64_t currentTime = HAL_GetFPGATime(&status);

    uint64_t delta = closestTrigger - currentTime;

    if (currentTime > closestTrigger) {
	if (was_active) {
    	    {
    		std::lock_guard<wpi::mutex> notifier_lock(notifier->mutex);
    		// closest trigger already expired; trigger callback directly
    		// this will cause the next closest trigger to be configured.
    		notifier->triggerTime = UINT64_MAX;
    		notifier->triggeredTime = currentTime;
    	    }
            notifier->cond.notify_all();
#ifdef ENABLE_ALARM_DEBUG
	    printf("ERROR:  HAL_UpdateNotifierAlarm:  Notifier is active, and currentTime (%llu) > closestTrigger(%llu).\n", currentTime, closestTrigger);
	    fflush(stdout);
#endif
        } else {
#ifdef ENABLE_ALARM_DEBUG
	    printf("ERROR:  HAL_UpdateNotifierAlarm:  Notifier NOT active, and currentTime (%llu) > closestTrigger(%llu).\n", currentTime, closestTrigger);
	    fflush(stdout);
#endif
	}
	// Current time is before closest trigger; retrigger notifier right way
	delta = 100;
    }

    // Configure one-shot timer to expire at next closest trigger time
    new_value.it_value.tv_sec  = delta / 1000000; // NUM_MICROSECS_PER_SEC
    new_value.it_value.tv_nsec = (delta % 1000000) * 1000; // NUM_MICROSECS_PER_SEC, converted to nanoseconds
    new_value.it_interval.tv_sec = 0;
    new_value.it_interval.tv_nsec = 0;

    // Enable the alarm (whether it already was active or not)
    if (timer_settime (*notifierAlarm, 0, &new_value, &old_value) == -1) {
        perror ("timer_settime in HAL_UpdateNotifierAlarm.");
#ifdef ENABLE_ALARM_DEBUG
        printf("currentTime:     %llu\n", currentTime);
        printf("closestTrigger:  %llu\n", closestTrigger);
        printf("delta:           %llu\n", delta);    
        printf("tv_sec:          %lu\n", new_value.it_value.tv_sec);
        printf("tv_nsec:         %lu\n", new_value.it_value.tv_nsec);
        printf("*notifierAlarm:  %p\n", *notifierAlarm);
#endif
    } else {
#ifdef ENABLE_ALARM_DEBUG
    	printf("HAL_UpdateNotifierAlarm:  Set new delta:  %llu for notifierAlarm %p\n", delta, *notifierAlarm);
    	fflush(stdout);
#endif
    }
  } else {
#ifdef ENABLE_ALARM_DEBUG
    printf("HAL_UpdateNotifierAlarm:  Trigger Time:  %llu is >= closestTrigger  %llu\n", triggerTime, closestTrigger);
#endif
  }
#ifdef ENABLE_ALARM_DEBUG
  printf("ERROR:  HAL_UpdateNotifierAlarm exiting.\n");
  fflush(stdout);
#endif
}

void HAL_CancelNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             int32_t* status) {
  auto notifier = notifierHandles->Get(notifierHandle);
  if (!notifier) return;

  {
    std::lock_guard<wpi::mutex> lock(notifier->mutex);
    notifier->triggerTime = UINT64_MAX;
  }
}

#if 0
// NOTE:  This version with timeout doesn't work in Java...
uint64_t HAL_WaitForNotifierAlarm(HAL_NotifierHandle notifierHandle,
                                  int32_t* status) {
  auto notifier = notifierHandles->Get(notifierHandle);
  if (!notifier) return 0;
  // Create a timeout, just in case of troubles
  auto timeoutTime = hal::fpga_clock::epoch()
			+ std::chrono::duration<double>(5.0);

  std::unique_lock<wpi::mutex> lock(notifier->mutex);
  bool wait_timeout = notifier->cond.wait_until(lock, timeoutTime, [&] {
    return !notifier->active || notifier->triggeredTime != UINT64_MAX;
  });
  if (wait_timeout) {
	  printf("Error!  HAL_WaitForNotifierAlarm() did not respond within the expected amount of time.\n");
	  fflush(stdout);
	  usleep(50000);
	  return 0;
  }
  return notifier->active ? notifier->triggeredTime : 0;
}
#endif
uint64_t HAL_WaitForNotifierAlarm(HAL_NotifierHandle notifierHandle,
                                  int32_t* status) {
  auto notifier = notifierHandles->Get(notifierHandle);
  if (!notifier) return 0;
  try {
    std::unique_lock<wpi::mutex> lock(notifier->mutex);
    notifier->cond.wait(lock, [&] {
      return !notifier->active || notifier->triggeredTime != UINT64_MAX;
    });
  } catch(...) {
	notifier->triggeredTime = 0;
  }
  return notifier->active ? notifier->triggeredTime : 0;
}
}  // extern "C"
