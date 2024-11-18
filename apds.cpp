#include "Arduino.h"
#include "apds.h"

APDS9500::APDS9500() {
    Wire.begin();
}

uint8_t APDS9500::init() {
	// Write, then read
	_buffer[0] = APDS9500_PartID_L;
	_buffer[1] = 0;

    // Request, read, and confirm chip ID
    Wire.beginTransmission(APDS9500_I2CADDR_DEFAULT);
    Wire.write(_buffer);
    Wire.endTransmission();

    Wire.requestFrom(APDS9500_I2CADDR_DEFAULT, 2);
    while(Wire.available()) {
        for(int i = 0; i < 2; i++) {
            _buffer[i] = Wire.read();
        }
    }

	if ((buffer[1] << 8 | buffer[0]) != APDS9500_CHIP_ID) {
		return 1;
	}

	/* Initialize Gesture Sensor */
	// Choose bank 0
	writeReg(APDS9500_R_RegBankSet, 0x00); 			// select bank 0

	// Define cursor limits
	/*writeReg(APDS9500_R_CursorClampLeft, 0x07);  		// min horiz value
	writeReg(APDS9500_R_CursorClampRight, 0x17); 		// max horiz value
	writeReg(APDS9500_R_CursorClampUp, 0x06);    		// min vert value
	writeReg(APDS9500_R_Int2_En, 0x01);          		// enable interrupt on proximity*/

	// Auto exposure/Auto gain Controls
	writeReg(APDS9500_R_AELedOff_UB, 0x2D); 			// exposure time upper bound
	writeReg(APDS9500_R_AELedOff_LB, 0x0F); 			// exposure time lower bound
	writeReg(APDS9500_R_AE_Exposure_UB_L, 0x3C); 		// low byte auto exposure upper bound
	writeReg(APDS9500_R_AE_Exposure_UB_H, 0x00); 		// high byte auto exposure upper bound
	writeReg(APDS9500_R_AE_Exposure_LB_L, 0x1E); 		// low byte auto exposure lower bound
	writeReg(0x48, 0x80); // R_AE_Exposure_UB [7:0]
	writeReg(0x49, 0x04); // R_AE_Exposure_UB [15:8]
	writeReg(0x4A, 0x40); // R_AE_Exposure_LB [7:0]
	writeReg(0x4B, 0x02); // R_AE_Exposure_LB [15:8]
	writeReg(APDS9500_R_AE_Gain_LB, 0x20); 			// auto gain upper bound
	writeReg(APDS9500_R_Manual, 0x10);     			// enable auto exposure
	writeReg(0x5E, 0x10);                  			// don't know
	writeReg(0x60, 0x27);                  			// don't know

	// Set up Interrupt
	writeReg(APDS9500_InputMode_GPIO_0_1, 0x42); 		// set GPIO0 as OUTPUT, GPIO1 as INPUT
	writeReg(APDS9500_InputMode_GPIO_2_3, 0x44); 		// set GPIO2 as INPUT, GPIO3 as INPUT
	writeReg(APDS9500_InputMode_INT, 0x04); 			// set INT as INPUT

	// Detection thresholds
	writeReg(APDS9500_R_Cursor_ObjectSizeTh, 0x01); 	// object size threshold for cursor mode
	writeReg(APDS9500_R_NoMotionCountThd, 0x06); 		// no motion counter threshold
	writeReg(APDS9500_R_ZDirectionThd, 0x0A);    		// gesture detection z threshold
	writeReg(APDS9500_R_ZDirectionXYThd, 0x0C); 		// gesture detection x and y thresholds
	writeReg(APDS9500_R_ZDirectionAngleThd, 0x05); 	// angle threshold for forward and backward detection
	writeReg(APDS9500_R_RotateXYThd, 0x14); 			// rotation detection threshold
	writeReg(APDS9500_R_Filter, 0x3F); 				// filter weight and frame position threshold
	writeReg(APDS9500_R_FilterImage, 0x19); 			// use pixel brightness for weak average filter
	writeReg(APDS9500_R_YtoZSum, 0x19);    			// z-direction mapping parameter
	writeReg(APDS9500_R_YtoZFactor, 0x0B); 			// z-direction mapping parameter
	writeReg(APDS9500_R_FilterLength, 0x03); 			// filter length for cursor object center
	writeReg(APDS9500_R_WaveThd, 0x64); 				// wave gesture counter and angle thresholds
	writeReg(APDS9500_R_AbortCountThd, 0x21); 			// abort gesture counter threshold

	// Change to Bank 1
	writeReg(APDS9500_R_RegBankSet, 0x01); 			// select bank 1

	//writeReg(0x32, 0x04);

	// Image size settings
	writeReg(APDS9500_Cmd_HStart, 0x0F); 				// horizontal starting point
	writeReg(APDS9500_Cmd_VStart, 0x10); 				// vertical starting point
	writeReg(APDS9500_Cmd_HV, 0x02);     				// vertical flip
	writeReg(APDS9500_R_LensShadingComp_EnH, 0x01); 	// enable lens shading compensation
	writeReg(APDS9500_R_Offest_Y, 0x39); 				// vertical offset of lens, set to 55
	writeReg(APDS9500_R_LSC, 0x7F);      				// Lens shading coefficient, set to 127
	writeReg(APDS9500_R_LSFT, 0x08);     				// shift amount, initialize to 10
	writeReg(0x3E, 0xFF);                				// don't know
	writeReg(0x5E, 0x3D);                				// don't know

	//current
	writeReg(0x32, 0x01);

	// Sleep mode parameters
	writeReg(APDS9500_R_IDLE_TIME_L, 0x96); 			// idle time low byte = 150 which is set for ~120 fps
	writeReg(APDS9500_R_IDLE_TIME_SLEEP_1_L, 0x97); 	// idle time for weak sleep, set for report rate ~ 120 Hz
	writeReg(APDS9500_R_IDLE_TIME_SLEEP_2_L, 0xCD); 	// idle time for deep sleep, low byte
	writeReg(APDS9500_R_IDLE_TIME_SLEEP_2_H, 0x01); 	// idle time for deep sleep, high byte
	writeReg(APDS9500_R_Object_TIME_2_L, 0x2C); 		// deep sleep enter time, low byte
	writeReg(APDS9500_R_Object_TIME_2_H, 0x01); 		// deep sleep enter time, high byte
	writeReg(APDS9500_R_TG_EnH, 0x01); 				// enable time gating
	writeReg(APDS9500_R_Auto_SLEEP_Mode, 0x35); 		// no object weak and deep sleep, object wake
	writeReg(APDS9500_R_Wake_Up_Sig_Sel, 0x00); 		// interrupt on time gate start

	// Start sensor
	writeReg(APDS9500_R_SRAM_Read_EnH, 0x01); 			// SRAM read enable
	writeReg(APDS9500_R_SPIOUT_EnH, 0x01);

	// Change back to bank 0 for data read
	writeReg(APDS9500_R_RegBankSet, 0x00); 			// select bank 0*/

	return 0;
}

uint8_t APDS9500::user_init() {
	writeReg(APDS9500_R_RegBankSet, 0x00); 			// select bank 0

	//writeReg(0x41, 0x00);
	//writeReg(0x42, 0x00);

	writeReg(APDS9500_R_ImageHeight, 0x1E);			// Set image height and width
	writeReg(APDS9500_R_ImageWidth, 0x1E);

	//writeReg(0x9F, 0x00);

	// Change to Bank 1
	writeReg(APDS9500_R_RegBankSet, 0x01); 			// select bank 1

	writeReg(0x02, 0x00);								// Set image horizontal start point
	writeReg(0x03, 0x00);								// Set image vertical start point

	writeReg(APDS9500_Cmd_HSize, 0x1E);				// Set image horizontal and vertical size
	writeReg(APDS9500_Cmd_VSize, 0x3C);

	writeReg(APDS9500_Cmd_HV, 0x99);					// Remove skips
	writeReg(0x1E, 0x00);

	writeReg(0x4A, 0x1E);
	writeReg(0x4B, 0x1E);

	writeReg(0x41, 0x40);

	// Enable SPI
	writeReg(APDS9500_R_SPIOUT_PXDNUM_L, 0x84);
	writeReg(APDS9500_R_SPIOUT_PXDNUM_H, 0x03);

	//writeReg(0x74, 0x35);
	//writeReg(0x74, 0x00);

	//writeReg(APDS9500_R_SRAM_Read_EnH, 0x00); 			// SRAM read enable
	writeReg(APDS9500_R_TG_EnH, 0x01); 				// enable time gating
	writeReg(APDS9500_R_SRAM_Read_EnH, 0x01); 			// SRAM read enable
	writeReg(APDS9500_R_SPIOUT_EnH, 0x01);

	// Change back to bank 0 for data read
	writeReg(APDS9500_R_RegBankSet, 0x00); 			// select bank 0

	return 0;
}

void APDS9500::selectRegBank(uint8_t reg) {
	buffer[0] = APDS9500_R_RegBankSet;
	buffer[1] = reg;
    Wire.beginTransmission(APDS9500_I2CADDR_DEFAULT);
    Wire.write(_buffer, 2);
    Wire.endTransmission();
}

void APDS9500::readReg(uint8_t addr, uint8_t *val) {
	HAL_I2C_Master_Transmit(hi2c, APDS9500_I2CADDR_DEFAULT<<1, &addr, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(hi2c, APDS9500_I2CADDR_DEFAULT<<1, val, 1, HAL_MAX_DELAY);

    Wire.beginTransmission(APDS9500_I2CADDR_DEFAULT);
    Wire.write(&addr);
    Wire.endTransmission();

    Wire.requestFrom(APDS9500_I2CADDR_DEFAULT, 1);
    while(Wire.available()) {
        *val = Wire.read();
    }
}

void APDS9500::writeReg(uint8_t addr, uint8_t val) {
	buffer[0] = addr;
	buffer[1] = val;
    Wire.beginTransmission(APDS9500_I2CADDR_DEFAULT);
    Wire.write(_buffer, 2);
    Wire.endTransmission();
}
