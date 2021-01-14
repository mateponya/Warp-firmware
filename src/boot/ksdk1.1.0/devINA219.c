// INA219 device drive

#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

uint16_t ICONFIG_BVOLTAGERANGE_16V =        (0x0000);  // 0-16V Range
uint16_t ICONFIG_GAIN_1_40MV        =       (0x0000);  // Gain 1, 40mV Range
uint16_t ICONFIG_BADCRES_12BIT       =      (0x0180);  // 12-bit bus res = 0..4097
uint16_t ICONFIG_SADCRES_12BIT_128S_69MS =  (0x0078);  // 128 x 12-bit shunt samples averaged together
uint16_t ICONFIG_MODE_SANDBVOLT_CONTINUOUS = (0x0007);
uint8_t INA219_CONFIG_REG = (0x00);
uint8_t INA219_CALIB_REG = (0x05);

extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;
uint16_t _calValue = 0;
uint8_t _currentDivider_mA = 0;

void
initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (0);
	return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload, uint16_t menuI2cPullupValue)
{
	uint8_t		payloadByte[2];
	uint8_t		commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x00: // Configuration register
		case 0x05: // Calibration register
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[0] = (uint8_t)((payload>>8) & 0xFF);
	payloadByte[1] = (uint8_t)(payload & 0xFF);
	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							2,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}




void setCalibration_16V_400mA(uint16_t menuI2cPullupValue) {

  // Calibration which uses the highest precision for
  // current measurement (0.1mA), at the expense of
  // only supporting 16V at 400mA max.

  // VBUS_MAX = 16V
  // VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
  // RSHUNT = 0.1               (Resistor value in ohms)

  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 0.4A

  // 2. Determine max expected current
  // MaxExpected_I = 0.4A

  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.0000122              (12uA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0.0000977              (98uA per bit)

  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.00005 (50uA per bit)

  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 8192 (0x2000)

  _calValue = 8192;

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.001 (1mW per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 1.63835A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // Max_Current_Before_Overflow = MaxPossible_I
  // Max_Current_Before_Overflow = 0.4
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.04V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If
  //
  // Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Max_ShuntVoltage_Before_Overflow = 0.04V

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 0.4 * 16V
  // MaximumPower = 6.4W

  // Set multipliers to convert raw current/power values
  _currentDivider_mA = 20;    // Current LSB = 50uA per bit (1000/50 = 20)
  //_powerMultiplier_mW = 1.0f; // Power LSB = 1mW per bit

  // Set Calibration register to 'Cal' calculated above
  //wireWriteRegister(REG_CALIBRATION, _calValue);
  WarpStatus    i2cWriteStatus1;
  i2cWriteStatus1 =  writeSensorRegisterINA219(INA219_CALIB_REG , /* Configuration register address */
                                               _calValue,
	                                       menuI2cPullupValue);


  // Set Config register to take into account the settings above
  uint16_t config = ICONFIG_BVOLTAGERANGE_16V |
                    ICONFIG_GAIN_1_40MV |
                    ICONFIG_BADCRES_12BIT |
                    ICONFIG_SADCRES_12BIT_128S_69MS |
                    ICONFIG_MODE_SANDBVOLT_CONTINUOUS;
  // Write data into configuration register
  // wireWriteRegister(REG_CONFIG, config);
  WarpStatus	i2cWriteStatus2;
  i2cWriteStatus2 =  writeSensorRegisterINA219(INA219_CONFIG_REG, /* Configuration register address */
					       config,
					       menuI2cPullupValue);
  return;
}

WarpStatus
readSensorRegisterMMA8451Q(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: // Configuration register
		case 0x01: // Shunt voltage register
		case 0x02: // Bus voltage register
		case 0x03: // Power register
		case 0x04: // Current register
		case 0x05: // Calibration register
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}


	i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceINA219State.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}


void
printSensorDataINA219(void)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;


	i2cReadStatus = readSensorRegisterINA219(0x04, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = (((readSensorRegisterValueMSB << 8) & 0xF0) | (readSensorRegisterValueLSB & 0x0F));
	readSensorRegisterValueCombined /= _currentDivider_mA;
	
	
	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined);
	}

	return;
}
