/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/Interrupts.h"

#include <memory>

#include <wpi/SafeThread.h>

#include "DigitalInternal.h"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"
#include "hal/handles/LimitedHandleResource.h"
#include "HALInitializer.h"
#include "PortsInternal.h"
#include "InterruptsInternal.h"
#include <VMXChannel.h>
#include <VMXResource.h>
#include <VMXIO.h>
#include <atomic>

using namespace hal;

static constexpr int64_t kHalWaitResultInterruptTimeout = 0x0;

class InterruptSafeThread : public wpi::SafeThread {
	void Main() {}
};

class BlockingInterruptManager: public wpi::detail::SafeThreadProxyBase {
public:
	BlockingInterruptManager(wpi::SafeThread *p_thread) :
			SafeThreadProxyBase(std::shared_ptr<wpi::SafeThread>(p_thread)) {
	}

	std::atomic<uint32_t> m_mask;

	// Invoked from interrupt handler
	void Notify(uint32_t mask) {
		m_mask = mask;
		if (m_thread != nullptr) {
			m_thread->m_cond.notify_one();
		}
	}

	int64_t Wait(uint64_t timeout_us) {
		std::unique_lock<wpi::mutex>& lock = GetLock();
		if (m_thread->m_cond.wait_for(lock, std::chrono::microseconds(timeout_us))
				== std::cv_status::timeout) {
			return kHalWaitResultInterruptTimeout;
		} else {
			return m_mask;
		}
	}
};

static void blockingInterruptHandler(uint32_t mask, void* param) {
	static_cast<BlockingInterruptManager*>(param)->Notify(mask);
}

struct InterruptResource {
	VMXResourceHandle vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(
			VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX);
	VMXChannelInfo vmx_chan_info;
	InterruptConfig interrupt_config = InterruptConfig(false);
	std::atomic<HAL_InterruptHandlerFunction> p_userHandler;
	void* param = nullptr;
	bool watcher = false;  // TODO:  Can this be removed?
	void Init() {
		vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(
				VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX);
		vmx_chan_info = VMXChannelInfo();
		interrupt_config = InterruptConfig(false);
		p_userHandler = nullptr;
		param = nullptr;
		watcher = false;
	}
};

static LimitedHandleResource<HAL_InterruptHandle, InterruptResource, kNumInterrupts,
		HAL_HandleEnum::Interrupt>* interruptHandles;

static std::atomic<InterruptResource *> handler_lookup_table[kNumInterrupts];
static std::atomic<uint8_t> hal_int_index_lookup_table[kNumVMXPiInterrupts];

static constexpr uint8_t kInvalidHALInterruptIndex = 255;

namespace hal {
namespace init {
void InitializeInterrupts() {
	static LimitedHandleResource<HAL_InterruptHandle, InterruptResource, kNumInterrupts,
			HAL_HandleEnum::Interrupt> iH;
	interruptHandles = &iH;
	for (int i = 0; i < kNumInterrupts; i++) {
		handler_lookup_table[i].store(nullptr);
	}
	for (int i = 0; i < kNumVMXPiInterrupts; i++) {
		hal_int_index_lookup_table[i] = kInvalidHALInterruptIndex;
	}
}
}  // namespace init
}  // namespace hal

/* This single function handles and dispatches all VMX-pi Interrupts */
static void VMXPi_InterruptHandler(uint32_t vmx_channel_index,
		InterruptEdgeType edge, void* param, uint64_t timestamp_us) {

	// Convert vmx_channel_index to wpi hal resource handle
	VMXResourceIndex interrupt_resource_index =
			static_cast<VMXResourceIndex>(vmx_channel_index);

	// get corresponding HAL_Interrupt object
	if (interrupt_resource_index >= kNumVMXPiInterrupts)
		return;

	uint8_t hal_int_index = hal_int_index_lookup_table[interrupt_resource_index];
	if (hal_int_index != kInvalidHALInterruptIndex) {
		InterruptResource *interrupt = handler_lookup_table[hal_int_index].load();
		if (interrupt != nullptr) {
			// The interrupt mask implemented in the WPI simulator HAL uses this mask format:
			// Bits 0-7:  If set, is rising edge interrupt.  Bit position indicates interrupt number (0-7)
			// Bits 8-15: If set, is falling edge interrupt.  Bit position indicates interrupt number (0-7)
			// NOTE:  This behavior limits the number of interrupts to 8
			HAL_InterruptHandlerFunction p_handler = interrupt->p_userHandler;
			if (p_handler) {
				uint32_t mask =
						(edge == InterruptEdgeType::RISING_EDGE_INTERRUPT) ?
								(1 << hal_int_index) :
								(1 << (hal_int_index + 8));
				p_handler(mask, interrupt->param);
			} else {
				// Interrupt handler invoked for interrupt w/no registered handlers
			}
		}
	}
}

extern "C" {

/**
 * API Function
 *
 * Allocates one of the interrupt handles (in the reference implementation, they are limited,
 * but on VMX-pi there is one for each VMXChannelIndex (both digital and analog).
 *
 * Note that at this point, the input channel is not yet known.
 *
 * If "watcher" is true, this interupt will be used in synchronous mode, where the user
 * program will block while waiting for the interrupt to occur.
 *
 * If "watcher" is false, this interrupt is be handled asynchronously by the user
 * program; in this case the user program must carefully manage re-entrancy.
 **/

HAL_InterruptHandle HAL_InitializeInterrupts(HAL_Bool watcher,
		int32_t* status) {
	hal::init::CheckInit();
	HAL_InterruptHandle handle = interruptHandles->Allocate();
	if (handle == HAL_kInvalidHandle) {
		*status = NO_AVAILABLE_RESOURCES;
		return HAL_kInvalidHandle;
	}

	auto anInterrupt = interruptHandles->Get(handle);
	if (anInterrupt == nullptr) {
		*status = NO_AVAILABLE_RESOURCES;
		return HAL_kInvalidHandle;
	}

	/* Since the input source channel is not yet known, */
	/* Defer VMX Resource allocation until later.       */
	anInterrupt->interrupt_config = InterruptConfig();
	anInterrupt->watcher = watcher;
	anInterrupt->p_userHandler = nullptr;
	anInterrupt->param = nullptr;

	return handle;
}

/**
 * API Function
 *
 * Cancel interrupts on this device.
 *
 * This deallocates all structures and disables any interrupts.
 **/
void* HAL_CleanInterrupts(HAL_InterruptHandle interruptHandle,
		int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		return nullptr;
	}

	int16_t hal_interrupt_handle_index = getHandleIndex(interruptHandle);
	if (hal_interrupt_handle_index >= kNumInterrupts) {
		*status = HAL_HANDLE_ERROR;
		return nullptr;
	}

	if (INVALID_VMX_RESOURCE_HANDLE(anInterrupt->vmx_res_handle)) {
		*status = VMXERR_IO_INVALID_RESOURCE_HANDLE;
		return nullptr;
	}

	VMXResourceIndex vmx_interrupt_res_index = EXTRACT_VMX_RESOURCE_INDEX(anInterrupt->vmx_res_handle);
	if (vmx_interrupt_res_index >= kNumVMXPiInterrupts) {
		*status = VMXERR_IO_INVALID_RESOURCE_INDEX;
		return nullptr;
	}

	HAL_DisableInterrupts(interruptHandle, status);

	mau::vmxIO->DeallocateResource(anInterrupt->vmx_res_handle, status);
	anInterrupt->Init();
	anInterrupt->p_userHandler = nullptr;
	anInterrupt->param = nullptr;

	handler_lookup_table[hal_interrupt_handle_index].store(nullptr);
	hal_int_index_lookup_table[vmx_interrupt_res_index] =
			kInvalidHALInterruptIndex;

	void* param = anInterrupt->param;
	interruptHandles->Free(interruptHandle);

	return param;
}

/**
 * In synchronous mode, wait for the defined interrupt to occur.
 * @param timeout Timeout in seconds
 * @param ignorePrevious If true, ignore interrupts that happened before
 * waitForInterrupt was called.
 * @return The mask of interrupts that fired:
 * 0x000:  Timeout
 * 0x001:  Rising Edge
 * 0x100:  Falling Edge
 * 0x101:  Both Rising and Falling Edge
 *
 * NOTE:  HAL_RequestInterrupts will have already been invoked before this.
 *
 */

int64_t HAL_WaitForInterrupt(HAL_InterruptHandle interruptHandle,
		double timeout, HAL_Bool ignorePrevious, int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	InterruptSafeThread ist; // encapsulates condition var, mutex and atomic "quit" boolean
	BlockingInterruptManager istproxy(&ist); // encapsulates mutex lock and SafeThread *

	// NOTE:  HAL_RequestInterrupts must have already been invoked previously.
	// Therefore, the blockingInterruptHandler may be invoked before the
	// following call to HAL_AttachInterruptHandler returns.
	HAL_AttachInterruptHandler(interruptHandle, blockingInterruptHandler,
			&istproxy, status);

	uint64_t timeout_us = static_cast<uint64_t>(timeout * 1e+6);
	int64_t result = istproxy.Wait(timeout_us);

	return result;
}

/**
 * Enable interrupts to occur on this input.
 * Interrupts are disabled when the RequestInterrupt call is made. This gives
 * time to do the setup of the other options before starting to field
 * interrupts.
 */
void HAL_EnableInterrupts(HAL_InterruptHandle interruptHandle,
		int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	mau::vmxIO->Interrupt_SetEnabled(anInterrupt->vmx_res_handle, true, status);
}

/**
 * Disable Interrupts without without deallocating structures.
 */
void HAL_DisableInterrupts(HAL_InterruptHandle interruptHandle,
		int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	mau::vmxIO->Interrupt_SetEnabled(anInterrupt->vmx_res_handle, false,
			status);
}

/**
 * Return the timestamp for the rising interrupt that occurred most recently.
 * This is in the same time domain as GetClock().
 * @return Timestamp in seconds since boot.
 */
int64_t HAL_ReadInterruptRisingTimestamp(HAL_InterruptHandle interruptHandle,
		int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	uint64_t timestamp_microseconds;
	if (!mau::vmxIO->Interrupt_GetLastRisingEdgeTimestampMicroseconds(
			anInterrupt->vmx_res_handle, timestamp_microseconds, status)) {
		return 0;
	}

	return timestamp_microseconds;
}

/**
 * Return the timestamp for the falling interrupt that occurred most recently.
 * This is in the same time domain as GetClock().
 * @return Timestamp in seconds since boot.
 */
int64_t HAL_ReadInterruptFallingTimestamp(HAL_InterruptHandle interruptHandle,
		int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	uint64_t timestamp_microseconds;
	if (!mau::vmxIO->Interrupt_GetLastFallingEdgeTimestampMicroseconds(
			anInterrupt->vmx_res_handle, timestamp_microseconds, status)) {
		return 0;
	}

	return timestamp_microseconds;
}

/**
 * API Function
 *
 * Attach an asynchronous interrupt handler to the interrupt handle
 * previously returned from HAL_RequestInterrupts.  This handler will
 * be invoked asynchronously when the interrupt fires.
 *
 * This function is also invoked for synchronous interrupt handling.
 *
 * wpiblic invokes this before HAL_EnableInterrupts is invoked.
 *
 * TODO:  Determine what the purpose of the analogTriggerType in this case is.
 */

void HAL_RequestInterrupts(HAL_InterruptHandle interruptHandle,
		HAL_Handle digitalSourceHandle, HAL_AnalogTriggerType analogTriggerType,
		int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	VMXChannelInfo vmx_src_channel_info;
	VMXResourceIndex vmx_interrupt_res_index;
	if (!GetVMXInterruptResourceIndexForDigitalSourceHandle(digitalSourceHandle,
			vmx_interrupt_res_index, vmx_src_channel_info, status)) {
		return;
	}

	if (vmx_interrupt_res_index >= kNumVMXPiInterrupts) {
		*status = VMXERR_IO_INVALID_RESOURCE_INDEX;
		return;
	}

	int16_t hal_interrupt_handle_index = getHandleIndex(interruptHandle);
	if (hal_interrupt_handle_index >= kNumInterrupts) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	hal_int_index_lookup_table[vmx_interrupt_res_index] =
			hal_interrupt_handle_index;
	handler_lookup_table[hal_interrupt_handle_index].store(anInterrupt.get());

	anInterrupt->interrupt_config.SetHandler(VMXPi_InterruptHandler);

	if (!mau::vmxIO->ActivateSinglechannelResource(vmx_src_channel_info,
			&anInterrupt->interrupt_config, anInterrupt->vmx_res_handle,
			status)) {
		handler_lookup_table[hal_interrupt_handle_index].store(nullptr);
		hal_int_index_lookup_table[vmx_interrupt_res_index] =
				kInvalidHALInterruptIndex;
		return;
	}
}

/**
 * API Function
 *
 * Attach an asynchronous interrupt handler to the interrupt handle
 * previously returned from HAL_RequestInterrupts.  This handler will
 * be invoked asynchronously when the interrupt fires.
 *
 * This function is not invoked for synchronous interrupt handling
 * (synchronous interrupts are received via HAL_WaitForInterrupt()).
 *
 * wpiblic invokes this before HAL_EnableInterrupts is invoked.
 */

void HAL_AttachInterruptHandler(HAL_InterruptHandle interruptHandle,
		HAL_InterruptHandlerFunction handler, void* param, int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}
	anInterrupt->param = param;
	anInterrupt->p_userHandler = handler;
}

/**
 * API Function
 *
 * This does not appear to be invoked anywhere in wpilibc, so is likely deprecated.
 *
 * The reference implementation (used here) creates and starts a thread,
 * performs synchronous interrupt handling via that thread.  When the interrupt
 * is recieved, this thread will invoke the provided handler.
 *
 * The purpose appears to be to defer a kernel-interrupt to a user-mode
 * interrupt handler on a different thread.  Since the VMX-pi HAL invokes
 * interrupts on a user-mode thread, this additional deferral of interrupt handling
 * is not necessary.  Therefore, the standard HAL_AttachInterruptHandler() is
 * invoked in this case.
 *
 * Note that this implies the handler executes on the VMX-pi HAL interrupt thread,
 * which implies that the handler should be implemented to execute efficiently.
 */

void HAL_AttachInterruptHandlerThreaded(HAL_InterruptHandle interrupt_handle,
		HAL_InterruptHandlerFunction handler, void* param, int32_t* status) {
	HAL_AttachInterruptHandler(interrupt_handle, handler, param, status);
}

/**
 * API Function
 *
 * Configures the previously-allocated interrupt handle to trigger on
 * rising edge, falling edge, or both edges.
 *
 * Per wpilibc, this API function must be set *before* HAL_RequestInterrupts.
 */

void HAL_SetInterruptUpSourceEdge(HAL_InterruptHandle interruptHandle,
		HAL_Bool risingEdge, HAL_Bool fallingEdge, int32_t* status) {
	auto interrupt = interruptHandles->Get(interruptHandle);
	if (interrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	if (risingEdge) {
		if (fallingEdge) {
			interrupt->interrupt_config.SetEdge(
					InterruptConfig::InterruptEdge::BOTH);
		} else {
			interrupt->interrupt_config.SetEdge(
					InterruptConfig::InterruptEdge::RISING);
		}
	} else {
		interrupt->interrupt_config.SetEdge(
				InterruptConfig::InterruptEdge::FALLING);
	}
}

}  // extern "C"
