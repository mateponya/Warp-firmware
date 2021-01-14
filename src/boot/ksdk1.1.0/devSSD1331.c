#include <stdint.h>

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

volatile uint8_t	inBuffer[1];
volatile uint8_t	payloadBytes[1];


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 13),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
};

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}


void writeNumber(uint16_t num);


int
devSSD1331init(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	enableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x06);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0x50);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);

	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);



	/*
	 *	Any post-initialization drawing commands go here.
	 */
	//...


	/*
	 *	Fill screen with green
	 */

	/*
	SEGGER_RTT_printf(0,"Green Color -----------------------\n");
	*/
	writeCommand(kSSD1331CommandDRAWRECT);
        writeCommand(0x00);
        writeCommand(0x00);
        writeCommand(0x5F);
        writeCommand(0x3F);
	writeCommand(0x00);	// line color C
	writeCommand(0xFF);	// line color B
	writeCommand(0x00);	// line color A
	writeCommand(0x00);	// fill color C
	writeCommand(0xFF);	// fill color B
	writeCommand(0x00);	// fill color A
	
	writeNumber(1234);

	return 0;
}

void drawLine(uint8_t col_start, uint8_t row_start, uint8_t col_end, uint8_t row_end, uint8_t R, uint8_t G, uint8_t B) {
    writeCommand(kSSD1331CommandDRAWLINE);
    writeCommand(col_start);	// Column Address of Start
    writeCommand(row_start);	// Row Address of Start
    writeCommand(col_end);	// Column Address of End
    writeCommand(row_end);	// Row Address of End
    writeCommand(B);     // Color C of the line
    writeCommand(G);     // Color B of the line
    writeCommand(B);     // Color A of the line
    return;
}
void drawRectangle(uint8_t col_start, uint8_t row_start, uint8_t col_end, uint8_t row_end, uint8_t R, uint8_t G, uint8_t B) {
    writeCommand(kSSD1331CommandDRAWRECT);
    writeCommand(col_start);	// Column Address of Start
    writeCommand(row_start);	// Row Address of Start
    writeCommand(col_end);	// Column Address of End
    writeCommand(row_end);	// Row Address of End
    writeCommand(B);     // Color C of the line
    writeCommand(G);     // Color B of the line
    writeCommand(R);     // Color A of the line
    writeCommand(B);     // Color C of the fill area
    writeCommand(G);     // Color B of the fill area
    writeCommand(R);     // Color A of the fill area
    return;
}

void drawZero(uint8_t col_x, uint8_t row_y, uint8_t R, uint8_t G, uint8_t B) {
    drawLine(col_x, row_y, col_x+10, row_y, R, G, B); // upper h line
    drawLine(col_x, row_y+20, col_x+10, row_y+20, R, G, B); // lower h line
    drawLine(col_x, row_y, col_x, row_y+20, R, G, B); // long left v line
    drawLine(col_x+10, row_y, col_x+10, row_y+20, R, G, B); // long right v line
    return;
}
void drawOne(uint8_t col_x, uint8_t row_y, uint8_t R, uint8_t G, uint8_t B) {
    drawLine(col_x+5, row_y, col_x+5, row_y+20, R, G, B); // v line
    return;
}
void drawTwo(uint8_t col_x, uint8_t row_y, uint8_t R, uint8_t G, uint8_t B) {
    drawLine(col_x, row_y, col_x+10, row_y, R, G, B); // upper h line
    drawLine(col_x, row_y+10, col_x+10, row_y+10, R, G, B); // centre h line
    drawLine(col_x, row_y+20, col_x+10, row_y+20, R, G, B); // lower h line
    drawLine(col_x, row_y+10, col_x, row_y+20, R, G, B); // lower left v line
    drawLine(col_x+10, row_y, col_x+10, row_y+10, R, G, B); // upper right v line
    return;
}
void drawThree(uint8_t col_x, uint8_t row_y, uint8_t R, uint8_t G, uint8_t B) {
    drawLine(col_x, row_y, col_x+10, row_y, R, G, B); // upper h line
    drawLine(col_x, row_y+10, col_x+10, row_y+10, R, G, B); // centre h line
    drawLine(col_x, row_y+20, col_x+10, row_y+20, R, G, B); // lower h line
    drawLine(col_x+10, row_y, col_x+10, row_y+20, R, G, B); // long right v line
    return;
}
void drawFour(uint8_t col_x, uint8_t row_y, uint8_t R, uint8_t G, uint8_t B) {
    drawLine(col_x, row_y+10, col_x+10, row_y+10, R, G, B); // centre h line
    drawLine(col_x, row_y, col_x, row_y+10, R, G, B); // left v line
    drawLine(col_x+10, row_y, col_x+10, row_y+20, R, G, B); // long right v line
    drawLine(col_x, row_y, col_x, row_y+10, R, G, B); // upper left v line
    return;
}
void drawFive(uint8_t col_x, uint8_t row_y, uint8_t R, uint8_t G, uint8_t B) {
    drawLine(col_x, row_y, col_x+10, row_y, R, G, B); // upper h line
    drawLine(col_x, row_y+10, col_x+10, row_y+10, R, G, B); // centre h line
    drawLine(col_x, row_y+20, col_x+10, row_y+20, R, G, B); // lower h line
    drawLine(col_x, row_y, col_x, row_y+10, R, G, B); // upper left v line
    drawLine(col_x+10, row_y+10, col_x+10, row_y+20, R, G, B); // lower right v line
    return;
}
void drawSix(uint8_t col_x, uint8_t row_y, uint8_t R, uint8_t G, uint8_t B) {
    drawLine(col_x, row_y, col_x+10, row_y, R, G, B); // upper h line
    drawLine(col_x, row_y+10, col_x+10, row_y+10, R, G, B); // centre h line
    drawLine(col_x, row_y+20, col_x+10, row_y+20, R, G, B); // lower h line
    drawLine(col_x, row_y, col_x, row_y+20, R, G, B); // long left v line
    drawLine(col_x+10, row_y+10, col_x+10, row_y+20, R, G, B); // lower right v line
    return;
}
void drawSeven(uint8_t col_x, uint8_t row_y, uint8_t R, uint8_t G, uint8_t B) {
    drawLine(col_x, row_y, col_x+10, row_y, R, G, B); // upper h line
    drawLine(col_x+10, row_y, col_x+10, row_y+20, R, G, B); // long right v line
    return;
}
void drawEight(uint8_t col_x, uint8_t row_y, uint8_t R, uint8_t G, uint8_t B) {
    drawLine(col_x, row_y, col_x+10, row_y, R, G, B); // upper h line
    drawLine(col_x, row_y+10, col_x+10, row_y+10, R, G, B); // centre h line
    drawLine(col_x, row_y+20, col_x+10, row_y+20, R, G, B); // lower h line
    drawLine(col_x, row_y, col_x, row_y+20, R, G, B); // long left v line
    drawLine(col_x+10, row_y, col_x+10, row_y+20, R, G, B); // long right v line
    return;
}
void drawNine(uint8_t col_x, uint8_t row_y, uint8_t R, uint8_t G, uint8_t B) {
    drawLine(col_x, row_y, col_x+10, row_y, R, G, B); // upper h line
    drawLine(col_x, row_y+10, col_x+10, row_y+10, R, G, B); // centre h line
    drawLine(col_x, row_y+20, col_x+10, row_y+20, R, G, B); // lower h line
    drawLine(col_x, row_y, col_x, row_y+10, R, G, B); // upper left v line
    drawLine(col_x+10, row_y, col_x+10, row_y+20, R, G, B); // long right v line
    return;
}

void writeCharacter(uint8_t character, uint8_t col_x, uint8_t row_y, uint8_t R, uint8_t G, uint8_t B) {
    switch(character) {
        case 0: {
            drawZero(col_x, row_y, R, G, B);
            break;
        }
        case 1: {
            drawOne(col_x, row_y, R, G, B);
            break;
        }
        case 2: {
            drawTwo(col_x, row_y, R, G, B);
            break;
        }
        case 3: {
            drawThree(col_x, row_y, R, G, B);
            break;
        }
        case 4: {
            drawFour(col_x, row_y, R, G, B);
            break;
        }
        case 5: {
            drawFive(col_x, row_y, R, G, B);
            break;
        }
        case 6: {
            drawSix(col_x, row_y, R, G, B);
            break;
        }
        case 7: {
            drawSeven(col_x, row_y, R, G, B);
            break;
        }
        case 8: {
            drawEight(col_x, row_y, R, G, B);
            break;
        }
        case 9: {
            drawNine(col_x, row_y, R, G, B);
            break;
        }
    }
    return;
}

void writeNumber(uint16_t num) {
    uint8_t R = 0;
    uint8_t G = 255;
    uint8_t B = 0;
    
    if(num < 500) {
        R = 255;
        G = 255;
        B = 0;
    }
    if(num < 200) {
        R = 255;
        G = 0;
        B = 0;
    }

    drawRectangle(0, 0, 95, 63, B, G, R);

    R = 255;
    G = 255;
    B = 255;

    uint8_t thousand = num/1000;
    num -= thousand*1000;
    uint8_t hundred = num/100;
    num -= hundred*100;
    uint8_t ten = num/10;
    num -= ten*10;
    uint8_t one = num;

    uint8_t col_x0 = 20; // x
    uint8_t row_y0 = 22; // y

    writeCharacter(thousand, col_x0, row_y0, R, G, B);
    writeCharacter(hundred, col_x0+14, row_y0, R, G, B);
    writeCharacter(ten, col_x0+28, row_y0, R, G, B);
    writeCharacter(one, col_x0+42, row_y0, R, G, B);

    return;
}
