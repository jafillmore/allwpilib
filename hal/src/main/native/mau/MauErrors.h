/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


#pragma once

#define MAU_CHANNEL_MAP_ERROR_MESSAGE					"Requested channel not present in VMX-pi Channel Map"
#define MAU_CHANNEL_MAP_ERROR					-21000

#define MAU_HICURRDIO_OUTPUTMODE_NOT_ENABLED_MESSAGE	"HighCurrentDIO Output Mode must be enabled"
#define MAU_HICURRDIO_OUTPUTMODE_NOT_ENABLED 	-21001

#define MAU_DIO_NOT_OUTPUT_CAPABLE_MESSAGE				"This Digital Channel is not output-capable"
#define MAU_DIO_NOT_OUTPUT_CAPABLE				-21002

#define MAU_DIO_ASSIGNED_TO_PWMGEN_MESSAGE				"This Digital Channel is currently routed to a PWM Generator"
#define MAU_DIO_ASSIGNED_TO_PWMGEN				-21003

// Mau error codes are negative values <= MAU_ERRNO_CODE_BASE
#define MAU_ERRNO_CODE_BASE	MAU_CHANNEL_MAP_ERROR

