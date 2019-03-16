/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/Counter.h"
#include "hal/Encoder.h"

#include "CounterInternal.h"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"
#include "hal/handles/LimitedHandleResource.h"
#include "HALInitializer.h"
#include "PortsInternal.h"
#include "MauInternal.h"
#include "DigitalInternal.h"

using namespace hal;

/*
 * VMX-pi HAL Counters are implemented using either the "Encoder" or "InputCapture" resources.
 * The underlying implementation is via STM32 Advanced and Basic Timers.
 *
 * NOTE:  In the future, Counters may be supported on the VMX-pi RPI Inputs
 *
 * The HAL Counters have three "counter" modes, and one "period" mode.
 *
 * Counter Mode 1:  HAL_Counter_kTwoPulse (UpDown)
 *   - This is implemented for VMX-pi using the InputCapture resource
 *   - NOTE:  Only sole "up source" or "down sources" configurations are supported
 *     (configurations with both up source and a down source are not supported;
 *      this is a limitation of the STM32 Timers
 * Counter Mode 2:  HAL_Counter_kExternalDirection
 *   - This is implemented for VMX-pi using the Encoder resource
 *   - The up source increments/decrements the count; the down source defines the direction
 * Counter Mode 3:  HAL_Counter_kPulseLength
 *   - This mode is not supported on VMX-pi
 *   - This exists primarily as an "advanced geartooth support" feature which was added to the WPI library at some point
 *   - In this model, the counter is incremented when pulses >= ThresholdPeriod occur
 *                  , the counter is decremented when pulses <  ThresholdPeriod occur
 * Period Mode 1:   HAL_Counter_kSemiperiod
 *   - This is implemented for VMX-pi using the InputCapture resource, in "Gated Mode"
 *   - In this mode, the period of the "active" pulse (the time between the "rising" and "falling" of the up source)
 *     is measured.
 *
 * Additional Notes:
 *   - AnalogTriggers may not be used as "up source" or "down source" currently with VMX-pi.
 *   - VMX-pi resource activation occurs when the up source and down source are set.
 */

namespace hal {

LimitedHandleResource<HAL_CounterHandle, CounterResource, kNumCounters,
                      HAL_HandleEnum::Counter>* counterHandles;
}  // namespace hal

namespace hal {
namespace init {
void InitializeCounter() {
  static LimitedHandleResource<HAL_CounterHandle, CounterResource, kNumCounters,
                               HAL_HandleEnum::Counter>
      cH;
  counterHandles = &cH;
}
}  // namespace init
}  // namespace hal


static bool AllocateVMXPiInputCapture(
		std::shared_ptr<CounterResource> counterResource,
		std::shared_ptr<DigitalPort> digPort, int32_t* status) {

	if (!mau::vmxIO->ActivateSinglechannelResource(digPort->vmx_chan_info,
			&counterResource->vmx_config, counterResource->vmx_res_handle,
			status)) {
		return false;
	}

	return true;
}

static bool IsVMXPiCounterAllocated(std::shared_ptr<CounterResource> counter) {
	return (!INVALID_VMX_RESOURCE_HANDLE(counter->vmx_res_handle) ||
			(counter->proxy_encoder_handle != HAL_kInvalidHandle));
}

// Deallocate all VMX-pi resources, including both counters and encoder resources
// This function must do everything possible to release resources, and it is
// assumed by all callers that it is successful.
static void DeallocateVMXPiInputCapture(std::shared_ptr<CounterResource> counter) {

	bool isActive = false;

	if (!INVALID_VMX_RESOURCE_HANDLE(counter->vmx_res_handle)) {
		mau::vmxIO->IsResourceActive(counter->vmx_res_handle, isActive,
			mau::vmxError);
		if (isActive) {
			mau::vmxIO->DeallocateResource(counter->vmx_res_handle, mau::vmxError);
		}
		counter->vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(
				VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX);
	}
	if (counter->proxy_encoder_handle != HAL_kInvalidHandle) {
		int32_t status;
		HAL_FreeEncoder(counter->proxy_encoder_handle, &status);
		counter->proxy_encoder_handle = HAL_kInvalidHandle;
	}
}

static bool VerifyCounterInputSource(std::shared_ptr<CounterResource> counterResource,
		HAL_Handle digitalSourceHandle,
		bool& a_input,
		int32_t *status) {

	// NOTE:  Unlike the reference implementation, VMX-pi does not
	// support analog triggers as counter input sources.
	HAL_HandleEnum counterSourceType = getHandleType(digitalSourceHandle);
	if ((counterSourceType == HAL_HandleEnum::AnalogInput) ||
		(counterSourceType == HAL_HandleEnum::AnalogTrigger)) {
		*status = MAU_COUNTER_ANALOG_INPUTS_UNSUPPORTED;
		return false;
	}

	// Retrieve the digital port
    auto portUp = digitalChannelHandles->Get(digitalSourceHandle, HAL_HandleEnum::DIO);
    if (portUp == nullptr) {
        *status = HAL_HANDLE_ERROR;
        return false;
    }

    if (counterResource->hal_counter_mode != HAL_Counter_kExternalDirection) {
		if (portUp->vmx_chan_info.capabilities & VMXChannelCapability::InputCaptureInput) {
			a_input = true;
		} else if (portUp->vmx_chan_info.capabilities & VMXChannelCapability::InputCaptureInput2) {
			a_input = false;
		} else {
			*status = MAU_DIO_CHANNEL_COUNTER_INCOMPATIBILITY;
			return false;
		}
    } else {
		if (portUp->vmx_chan_info.capabilities & VMXChannelCapability::EncoderAInput) {
			a_input = true;
		} else if (portUp->vmx_chan_info.capabilities & VMXChannelCapability::EncoderBInput) {
			a_input = false;
    	} else {
			*status = MAU_DIO_CHANNEL_ENCODER_INCOMPATIBILITY;
			return false;
		}
    }

	return true;
}

// Configures the VMX-pi "InputCaptureConfig" object with the currently-requested settings.
// NOTE:  This function does not take any action in ExternalDirection mode.
static bool ConfigureInputCapture(InputCaptureConfig& config,
		bool count_up,
		InputCaptureConfig::CaptureChannel input_channel,
		InputCaptureConfig::CaptureChannelActiveEdge active_edge,
		int32_t samplesToAverage,
		HAL_Counter_Mode mode) {

	if (mode == HAL_Counter_kTwoPulse) {

		// TwoPulse Mode: Input capture w/internal clock, triggered by active edge of input source
		// The counter value is accessed to determine the total pulse count

		/**** CLOCK CONFIGURATION ***/
		if (input_channel == InputCaptureConfig::CaptureChannel::CH1) {
			config.SetCounterClockSource(InputCaptureConfig::CounterClockSource::FILTERED_CH1);
		} else {
			config.SetCounterClockSource(InputCaptureConfig::CounterClockSource::FILTERED_CH2);
		}

		if (count_up) {
			config.SetCounterDirection(InputCaptureConfig::CounterDirection::DIRECTION_UP);
		} else {
			config.SetCounterDirection(InputCaptureConfig::CounterDirection::DIRECTION_DN);
		}

		/**** SOURCE CONFIGURATION ***/
		// NOTE:  CaptureChannelSource is Dynamic, meaning it is dynamically selected during Resource Activation.  */
		config.SetCaptureChannelSource(input_channel, InputCaptureConfig::CaptureChannelSource::CAPTURE_SIGNAL_DYNAMIC);
		// Specify active edge
		config.SetCaptureChannelActiveEdge(input_channel, active_edge);
		// Set Prescaler to 1
		config.SetCaptureChannelPrescaler(input_channel, InputCaptureConfig::CaptureChannelPrescaler::x1);

		/**** SLAVE MODE CONFIGURATION ***/
		config.SetSlaveMode(InputCaptureConfig::SlaveMode::SLAVEMODE_DISABLED);

		/**** CAPTURE FILTER CONFIGURATION ***/
		uint8_t filterNumber = config.GetClosestCaptureCaptureFilterNumSamples(samplesToAverage);
		config.SetCaptureChannelFilter(input_channel, filterNumber);

		return true;

	} else if (mode == HAL_Counter_kSemiperiod) {

		// SemiPeriod Mode: configure input capture w/internal clock, gated on the up source input
		// When the trigger occurs, the counter value increments based on the internal clock
		// The gate opens when the up source rising edge occurs, and closes when the up source falling edge occurs

		// The usage model for this mode is defined within the WPI Library Ultrasonic class:
		//   (a) SetSemiPeriodMode()
		//   (b) Reset() [resets the counter]
		//   (c) Get() [retrieves the value of the counter]

		/**** CLOCK CONFIGURATION ***/
		config.SetCounterClockSource(InputCaptureConfig::CounterClockSource::INTERNAL);

		/**** SOURCE CONFIGURATION ***/
		// NOTE:  CaptureChannelSource is Dynamic, meaning it is dynamically selected during Resource Activation.  */
		config.SetCaptureChannelSource(input_channel, InputCaptureConfig::CaptureChannelSource::CAPTURE_SIGNAL_DYNAMIC);
		// Select Rising Edge to match whether this is the up or down source
		config.SetCaptureChannelActiveEdge(input_channel, active_edge);
		// Set Prescaler to 1
		config.SetCaptureChannelPrescaler(input_channel, InputCaptureConfig::CaptureChannelPrescaler::x1);

		/**** SLAVE MODE CONFIGURATION ***/
		// Note CounterDirection is only used in Encoder mode.
		// Use dynamic trigger source (first-routed port)
		config.SetSlaveMode(InputCaptureConfig::SlaveMode::SLAVEMODE_GATED);
		config.SetSlaveModeTriggerSource(InputCaptureConfig::SlaveModeTriggerSource::TRIGGER_DYNAMIC);

		/**** CAPTURE FILTER CONFIGURATION ***/
		uint8_t filterNumber = config.GetClosestCaptureCaptureFilterNumSamples(samplesToAverage);
		config.SetCaptureChannelFilter(input_channel, filterNumber);

		return true;
	}

	return false;
}

/* Allocate/Activate the VMX-pi resources for this counter; if already allocated,
 * those existing resources are first deallocated.  Supports allocation of either
 * VMX-pi InputCapture or Encoder resources, depending upon the current Counter mode.
 */
static bool AllocReallocCounter(std::shared_ptr<CounterResource> counter, bool error_if_dual_extdirection_sources_not_present, int32_t *status) {

	DeallocateVMXPiInputCapture(counter);

	bool up_source_valid = (counter->up_source_digital_handle != HAL_kInvalidHandle);
	bool down_source_valid = (counter->down_source_digital_handle != HAL_kInvalidHandle);

	if (counter->hal_counter_mode != HAL_Counter_kExternalDirection) {

		if (up_source_valid && down_source_valid) {
			*status = MAU_COUNTER_DUAL_INPUTS_UNSUPPORTED;
			return false;
		}

		// non-ExternalDirection modes are implemented on VMX-pi using an InputCapture resource.
		if ((counter->hal_counter_mode == HAL_Counter_kTwoPulse) ||
			(counter->hal_counter_mode == HAL_Counter_kSemiperiod)) {

			HAL_Handle sourceHandle = up_source_valid ? counter->up_source_digital_handle : counter->down_source_digital_handle;
			InputCaptureConfig::CaptureChannelActiveEdge active_edge = up_source_valid ?
					counter->up_source_active_edge :
					counter->down_source_active_edge;

			bool a_input;
			if (!VerifyCounterInputSource(counter, sourceHandle, a_input, status)) {
				return false;
			}

			InputCaptureConfig::CaptureChannel capture_channel = a_input ?
					InputCaptureConfig::CaptureChannel::CH1 :
					InputCaptureConfig::CaptureChannel::CH2;

			if (!ConfigureInputCapture(counter->vmx_config, up_source_valid, capture_channel, active_edge, counter->samplesToAverage, counter->hal_counter_mode)) {
				return false;
			}

		    auto portSource = digitalChannelHandles->Get(sourceHandle, HAL_HandleEnum::DIO);
		    if (portSource == nullptr) {
		        *status = HAL_HANDLE_ERROR;
		        return false;
		    }

			if (!AllocateVMXPiInputCapture(counter, portSource, status)) {
				return false;
			}
		}

	} else {   // ExternalDirection mode:  implemented using an Encoder resource.  Two input channels are required.

		if (!up_source_valid || !down_source_valid) {
			if (error_if_dual_extdirection_sources_not_present) {
				*status = MAU_COUNTER_DUAL_INPUTS_REQUIRED;
			} else {
				// This is treated as a "silent error" case, since this function
				// can be invoked before both sources are present.
			}
			return false;
		}

		// TODO:  Is there ever a case for swapping these handles (e.g., reverse direction mode)
		HAL_Handle a_source_handle = counter->up_source_digital_handle;
		HAL_Handle b_source_handle = counter->down_source_digital_handle;

		auto port_a = digitalChannelHandles->Get(a_source_handle, HAL_HandleEnum::DIO);
		auto port_b = digitalChannelHandles->Get(b_source_handle, HAL_HandleEnum::DIO);

		if ((port_a == nullptr) || (port_b == nullptr)) {
			*status = HAL_HANDLE_ERROR;
			return false;
		}

		// Allocate an Encoder with both a and b sources.
		// Note that WPI counters only support x1 and x2 modes.
		// If the Up source active edge is BOTH, x2 mode is selected.
		HAL_EncoderEncodingType encoding_type =
				(counter->up_source_active_edge == InputCaptureConfig::CaptureChannelActiveEdge::ACTIVE_BOTH) ?
						HAL_EncoderEncodingType::HAL_Encoder_k2X :
						HAL_EncoderEncodingType::HAL_Encoder_k1X;
		HAL_EncoderHandle encoderHandle = HAL_InitializeEncoder(
				a_source_handle,
				HAL_AnalogTriggerType::HAL_Trigger_kInWindow,
				b_source_handle,
				HAL_AnalogTriggerType::HAL_Trigger_kInWindow,
				counter->reverse_direction,
				encoding_type,
				status);
		counter->proxy_encoder_handle = encoderHandle;

	}

	return true;

}


extern "C" {
/* mode:
 *       HAL_Counter_kExternalDirection:  In this case, the VMX-pi resource is not
 *       activated until both up and down sources are set
 *
 *       HAL_Counter_kTwoPulse:  In this case, only one source (up or down) is supported.
 *       In this case, the VMX-pi resource is not activated until either the
 *       up or down source is set.
 *
 *       HAL_Counter_kPulseLength:  In this case, only one source (up or down) is supported.
 *       In this case, the VMX-pi resource is not activated until either the
 *       up or down source is set.
 *
 *       HAL_Counter_kPulseLength:  This case (direction changes based upon length
 *       of last pulse) is not currently supported on VMX-pi.
 */
HAL_CounterHandle HAL_InitializeCounter(HAL_Counter_Mode mode, int32_t* index,
		int32_t* status) {
	hal::init::CheckInit();

	if (mode == HAL_Counter_Mode::HAL_Counter_kPulseLength) {
		*status = MAU_COUNTER_PULSELENGTH_MODE_UNSUPPORTED;
		return HAL_kInvalidHandle;
	}

	auto handle = counterHandles->Allocate();
	if (handle == HAL_kInvalidHandle) {
		*status = NO_AVAILABLE_RESOURCES;
		return HAL_kInvalidHandle;
	}

	auto counter = counterHandles->Get(handle);
	if (counter == nullptr) {  // would only occur on thread issue
		counterHandles->Free(handle);
		*status = HAL_HANDLE_ERROR;
		return HAL_kInvalidHandle;
	}

	*counter.get() = CounterResource();

	counter->index = *index = getHandleIndex(handle);;
	counter->hal_counter_mode = mode;

	// VMX-pi Counter resources are not allocated until later, when up/down source edges are set

	return handle;
}

void HAL_FreeCounter(HAL_CounterHandle counterHandle, int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter != nullptr) {
		DeallocateVMXPiInputCapture(counter);
	}

	*counter.get() = CounterResource();
	counterHandles->Free(counterHandle);

}

/* Counters in ExternalDirection mode (e.g., encoders) have their
 * "AverageSize" set to either 1 (for 1x) or 2 (for 2x).
 */
void HAL_SetCounterAverageSize(HAL_CounterHandle counterHandle, int32_t size,
                               int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter != nullptr) {

		counter->samplesToAverage = size;

		if (IsVMXPiCounterAllocated(counter)) {
			AllocReallocCounter(counter, true, status);
		}
	}

}

/* Sets up source for counter.  This implies an underlying selection
 * of the A or B STM32 timer input, based upon channel index.
 *
 * Note that if a downsource has already been set, an error occurs unless
 * the counter is configured for ExternalDirection (e.g., encoder) mode.
 */
void HAL_SetCounterUpSource(HAL_CounterHandle counterHandle,
                            HAL_Handle digitalSourceHandle,
                            HAL_AnalogTriggerType analogTriggerType, /* Unused */
                            int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	bool a_input;
	if (!VerifyCounterInputSource(counter, digitalSourceHandle, a_input, status)) {
		return;
	}

	InputCaptureConfigBase::CaptureChannel input_channel = a_input ?
			InputCaptureConfigBase::CaptureChannel::CH1 :
			InputCaptureConfigBase::CaptureChannel::CH2;

	ConfigureInputCapture(counter->vmx_config,
			true, /* Count up */
			input_channel,
			counter->up_source_active_edge,
			counter->samplesToAverage,
			counter->hal_counter_mode);

	AllocReallocCounter(counter, false, status);
}

/**
 * Set the edge sensitivity on an up counting source.
 *
 * Set the up source to either detect rising edges or falling edges or both.
 *
 * @param risingEdge  True to trigger on rising edges
 * @param fallingEdge True to trigger on falling edges
 *
 * NOTE:  This can be invoked ~after~ SetCounterUpSource/SetCounterDownSource
 *
 * NOTE:  This is only invoked in WPI library for ExternalDirection (Encoder) mode.
 */
void HAL_SetCounterUpSourceEdge(HAL_CounterHandle counterHandle,
                                HAL_Bool risingEdge, HAL_Bool fallingEdge,
                                int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	if (risingEdge) {
		if (fallingEdge) {
			counter->up_source_active_edge = InputCaptureConfig::CaptureChannelActiveEdge::ACTIVE_BOTH;
		} else {
			counter->up_source_active_edge = InputCaptureConfig::CaptureChannelActiveEdge::ACTIVE_RISING;
		}
	} else if (fallingEdge) {
		counter->up_source_active_edge = InputCaptureConfig::CaptureChannelActiveEdge::ACTIVE_FALLING;
	}

	if (IsVMXPiCounterAllocated(counter)) {
		AllocReallocCounter(counter, false, status);
	}

}

void HAL_ClearCounterUpSource(HAL_CounterHandle counterHandle,
                              int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	counter->up_source_digital_handle = HAL_kInvalidHandle;

	if (IsVMXPiCounterAllocated(counter)) {
		AllocReallocCounter(counter, true, status);
	}

}

/* Sets down source for counter.  This implies an underlying selection
 * of the A or B STM32 timer input, based upon channel index.
 *
 * Note that if a up source has already been set, an error occurs.
 */
void HAL_SetCounterDownSource(HAL_CounterHandle counterHandle,
                              HAL_Handle digitalSourceHandle,
                              HAL_AnalogTriggerType analogTriggerType,
                              int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	bool a_input;
	if (!VerifyCounterInputSource(counter, digitalSourceHandle, a_input, status)) {
		return;
	}

	InputCaptureConfigBase::CaptureChannel input_channel = a_input ?
			InputCaptureConfigBase::CaptureChannel::CH1 :
			InputCaptureConfigBase::CaptureChannel::CH2;

	ConfigureInputCapture(counter->vmx_config,
			false, /* Count Down */
			input_channel,
			counter->down_source_active_edge,
			counter->samplesToAverage,
			counter->hal_counter_mode);

	AllocReallocCounter(counter, false, status);

}

void HAL_SetCounterDownSourceEdge(HAL_CounterHandle counterHandle,
                                  HAL_Bool risingEdge, HAL_Bool fallingEdge,
                                  int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	if (risingEdge) {
		if (fallingEdge) {
			counter->down_source_active_edge = InputCaptureConfig::CaptureChannelActiveEdge::ACTIVE_BOTH;
		} else {
			counter->down_source_active_edge = InputCaptureConfig::CaptureChannelActiveEdge::ACTIVE_RISING;
		}
	} else if (fallingEdge) {
		counter->down_source_active_edge = InputCaptureConfig::CaptureChannelActiveEdge::ACTIVE_FALLING;
	}

	if (IsVMXPiCounterAllocated(counter)) {
		AllocReallocCounter(counter, true, status);
	}

}

void HAL_ClearCounterDownSource(HAL_CounterHandle counterHandle,
                                int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	counter->down_source_digital_handle = HAL_kInvalidHandle;

	if (IsVMXPiCounterAllocated(counter)) {
		AllocReallocCounter(counter, true, status);
	}

}

void HAL_SetCounterUpDownMode(HAL_CounterHandle counterHandle,
                              int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	counter->hal_counter_mode = HAL_Counter_Mode::HAL_Counter_kTwoPulse;

	if (IsVMXPiCounterAllocated(counter)) {
		AllocReallocCounter(counter, false, status);
	}

}

void HAL_SetCounterExternalDirectionMode(HAL_CounterHandle counterHandle,
                                         int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	counter->hal_counter_mode = HAL_Counter_Mode::HAL_Counter_kExternalDirection;

	if (IsVMXPiCounterAllocated(counter)) {
		AllocReallocCounter(counter, true, status);
	}

}

/* This is invoked in the case of Ultrasonic sensor after the counter
 * has previously been configured to TwoPulse mode and the input
 * channel allocated.
 */
void HAL_SetCounterSemiPeriodMode(HAL_CounterHandle counterHandle,
                                  HAL_Bool highSemiPeriod, int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	counter->hal_counter_mode = HAL_Counter_Mode::HAL_Counter_kSemiperiod;

	if (IsVMXPiCounterAllocated(counter)) {
		AllocReallocCounter(counter, false, status);
	}

}

/* This is invoked in the case of GearTooth sensor after the counter
 * has previously been configured to TwoPulse mode and the input
 * channel allocated.
 *
 * NOTE:  This is not currently supported by VMX-pi counters.
 */
void HAL_SetCounterPulseLengthMode(HAL_CounterHandle counterHandle,
                                   double threshold, int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	*status = MAU_COUNTER_PULSELENGTH_MODE_UNSUPPORTED;
	return;
}

int32_t HAL_GetCounterSamplesToAverage(HAL_CounterHandle counterHandle,
                                       int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 1;
	}

	return counter->samplesToAverage;
}

/* Configures Digital Filter of input sources. */
void HAL_SetCounterSamplesToAverage(HAL_CounterHandle counterHandle,
                                    int32_t samplesToAverage, int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	counter->samplesToAverage = samplesToAverage;
}

/**
 * Reset the Counter to zero.
 * Set the counter value to zero. This doesn't effect the running state of the
 * counter, just sets the current value to zero.
 */
void HAL_ResetCounter(HAL_CounterHandle counterHandle, int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	if (!INVALID_VMX_RESOURCE_HANDLE(counter->vmx_res_handle)) {
		mau::vmxIO->InputCapture_Reset(counter->vmx_res_handle, status);
	} else if (counter->proxy_encoder_handle != HAL_kInvalidHandle){
		HAL_ResetEncoder(counter->proxy_encoder_handle, status);
	}

}

int32_t HAL_GetCounter(HAL_CounterHandle counterHandle, int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	int32_t counter_value = 0;

	if (!INVALID_VMX_RESOURCE_HANDLE(counter->vmx_res_handle)) {
		mau::vmxIO->InputCapture_GetCount(counter->vmx_res_handle, counter_value, status);
		if (counter->hal_counter_mode == HAL_Counter_Mode::HAL_Counter_kSemiperiod) {
			if (counter->update_when_empty) {
				if (HAL_GetCounterStopped(counterHandle,status)) {
					counter_value = 0;
				}
			}
		}
	} else if (counter->proxy_encoder_handle != HAL_kInvalidHandle){
		counter_value = HAL_GetEncoder(counter->proxy_encoder_handle, status);
	}

	return counter_value;
}

/* The period here is in units of seconds. */
/* This value only makes sense when the counter is in HAL_Counter_kSemiperiod mode. */
double HAL_GetCounterPeriod(HAL_CounterHandle counterHandle, int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0.0;
	}

	if (counter->hal_counter_mode != HAL_Counter_kSemiperiod) {
		*status = MAU_COUNTER_SEMIPERIOD_MODE_REQUIRED;
		return 0.0;
	}

	int32_t tick_count = HAL_GetCounter(counterHandle, status);

	// Convert tick count (in microseconds) to seconds.

	return tick_count * 1e+6;
}

/* This is invoked in the case of Ultrasonic sensor after the counter
 * has previously been configured to TwoPulse mode and the input
 * channel allocated.
 */
void HAL_SetCounterMaxPeriod(HAL_CounterHandle counterHandle, double maxPeriod,
                             int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	uint8_t stall_period_20ms_periods = static_cast<uint8_t>(maxPeriod * (1000 / 20));

	counter->vmx_config.SetStallTimeout20MsPeriods(stall_period_20ms_periods);

	if (counter->proxy_encoder_handle != HAL_kInvalidHandle) {
		HAL_SetEncoderMaxPeriod(counter->proxy_encoder_handle, maxPeriod, status);
	} else {
		if (IsVMXPiCounterAllocated(counter)) {
			AllocReallocCounter(counter, true, status);
		}
	}
}

/* Theoretically, this only makes sense when the counter is in HAL_Counter_kSemiperiod mode. */
void HAL_SetCounterUpdateWhenEmpty(HAL_CounterHandle counterHandle,
                                   HAL_Bool enabled, int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	counter->update_when_empty = enabled;

}

HAL_Bool HAL_GetCounterStopped(HAL_CounterHandle counterHandle,
                               int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return false;
	}

	if (!INVALID_VMX_RESOURCE_HANDLE(counter->vmx_res_handle)) {
		bool forward_direction;
		bool active = false;
		mau::vmxIO->InputCapture_InputStatus(counter->vmx_res_handle, forward_direction, active, status);
		return active;
	} else if (counter->proxy_encoder_handle != HAL_kInvalidHandle) {
		return HAL_GetEncoderStopped(counter->proxy_encoder_handle, status);
	} else {
		return true;
	}

}

HAL_Bool HAL_GetCounterDirection(HAL_CounterHandle counterHandle,
                                 int32_t* status) {

	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return false;
	}

	if (!INVALID_VMX_RESOURCE_HANDLE(counter->vmx_res_handle)) {
		bool forward_direction;
		bool active = false;
		mau::vmxIO->InputCapture_InputStatus(counter->vmx_res_handle, forward_direction, active, status);
		return forward_direction;
	} else if (counter->proxy_encoder_handle != HAL_kInvalidHandle) {
		return HAL_GetEncoderDirection(counter->proxy_encoder_handle, status);
	} else {
		return true;
	}

}

/* This is only supported in ExternalDirection (Encoder) mode. */
void HAL_SetCounterReverseDirection(HAL_CounterHandle counterHandle,
                                    HAL_Bool reverseDirection,
                                    int32_t* status) {
	auto counter = counterHandles->Get(counterHandle);
	if (counter == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	if (counter->hal_counter_mode != HAL_Counter_Mode::HAL_Counter_kExternalDirection) {
		*status = MAU_COUNTER_EXT_DIRECTION_MODE_REQUIRED;
		return;
	}

	if (IsVMXPiCounterAllocated(counter)) {
		HAL_SetEncoderReverseDirection(counter->proxy_encoder_handle, reverseDirection, status);
	} else {
		counter->reverse_direction = reverseDirection;
	}

}
}  // extern "C"
