#ifndef __LL_SPI_ILI9341_H
#define __LL_SPI_ILI9341_H
//-------------------------------------------------------------------
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_rng.h"
#include "stm32f4xx_ll_dma.h"
#include <stdio.h>
#include <stdlib.h>
#include "fonts.h"
//-------------------------------------------------------------------
#define RESET_ACTIVE() LL_GPIO_ResetOutputPin(GPIOD,LL_GPIO_PIN_12)
#define RESET_IDLE() LL_GPIO_SetOutputPin(GPIOD,LL_GPIO_PIN_12)
#define CS_ACTIVE() LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_2)
#define CS_DESELECT() LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_2)
#define DC_SET_SEND_COMMAND() LL_GPIO_ResetOutputPin(GPIOD,LL_GPIO_PIN_13)
#define DC_SET_SEND_DATA() LL_GPIO_SetOutputPin(GPIOD,LL_GPIO_PIN_13)
//-------------------------------------------------------------------
#define TFT9341_MADCTL_MY  0x80
#define TFT9341_MADCTL_MX  0x40
#define TFT9341_MADCTL_MV  0x20
#define TFT9341_MADCTL_ML  0x10
#define TFT9341_MADCTL_RGB 0x00
#define TFT9341_MADCTL_BGR 0x08
#define TFT9341_MADCTL_MH  0x04
//--------------------------------------------------------------------
#define ORIENTATION_PORTRAIT 			0x48
#define ORIENTATION_LANDSCAPE 			0x28
#define ORIENTATION_PORTRAIT_MIRROR 	0x88
#define ORIENTATION_LANDSCAPE_MIRROR 	0xE8
//--------------------------------------------------------------------
#define TFT_ILI9341_ROTATION (TFT9341_MADCTL_MX | TFT9341_MADCTL_BGR)
//--------------------------------------------------------------------
//COLOR DEFINITION
#define TFT_COLOR_ILI9341_BLACK 	0x0000       	///<   0,   0,   0
#define TFT_COLOR_ILI9341_NAVY 		0x000F        	///<   0,   0, 123
#define TFT_COLOR_ILI9341_DARKGREEN 0x03E0   		///<   0, 125,   0
#define TFT_COLOR_ILI9341_DARKCYAN 	0x03EF    		///<   0, 125, 123
#define TFT_COLOR_ILI9341_MAROON 	0x7800      	///< 123,   0,   0
#define TFT_COLOR_ILI9341_PURPLE 	0x780F      	///< 123,   0, 123
#define TFT_COLOR_ILI9341_OLIVE 	0x7BE0       	///< 123, 125,   0
#define TFT_COLOR_ILI9341_LIGHTGREY 0xC618   		///< 198, 195, 198
#define TFT_COLOR_ILI9341_DARKGREY 	0x7BEF    		///< 123, 125, 123
#define TFT_COLOR_ILI9341_BLUE 		0x001F        	///<   0,   0, 255
#define TFT_COLOR_ILI9341_GREEN 	0x07E0       	///<   0, 255,   0
#define TFT_COLOR_ILI9341_CYAN 		0x07FF        	///<   0, 255, 255
#define TFT_COLOR_ILI9341_RED 		0xF800         	///< 255,   0,   0
#define TFT_COLOR_ILI9341_MAGENTA 	0xF81F     		///< 255,   0, 255
#define TFT_COLOR_ILI9341_YELLOW 	0xFFE0      	///< 255, 255,   0
#define TFT_COLOR_ILI9341_WHITE 	0xFFFF       	///< 255, 255, 255
#define TFT_COLOR_ILI9341_ORANGE 	0xFD20      	///< 255, 165,   0
#define TFT_COLOR_ILI9341_GREENYELLOW 0xAFE5 		///< 173, 255,  41
#define TFT_COLOR_ILI9341_PINK 		0xFC18        	///< 255, 130, 198
//--------------------------------------------------------------------
//COMMAND DEFINITION
#define TFT_ILI9341_SWRESET             0x01   /* Software Reset */
#define TFT_ILI9341_READ_DISPLAY_ID     0x04   /* Read display identification information */
#define TFT_ILI9341_RDDST               0x09   /* Read Display Status */
#define TFT_ILI9341_RDDPM               0x0A   /* Read Display Power Mode */
#define TFT_ILI9341_RDDMADCTL           0x0B   /* Read Display MADCTL */
#define TFT_ILI9341_RDDCOLMOD           0x0C   /* Read Display Pixel Format */
#define TFT_ILI9341_RDDIM               0x0D   /* Read Display Image Format */
#define TFT_ILI9341_RDDSM               0x0E   /* Read Display Signal Mode */
#define TFT_ILI9341_RDDSDR              0x0F   /* Read Display Self-Diagnostic Result */
#define TFT_ILI9341_SPLIN               0x10   /* Enter Sleep Mode */
#define TFT_ILI9341_SLEEP_OUT           0x11   /* Sleep out register */
#define TFT_ILI9341_PTLON               0x12   /* Partial Mode ON */
#define TFT_ILI9341_NORMAL_MODE_ON      0x13   /* Normal Display Mode ON */
#define TFT_ILI9341_DINVOFF             0x20   /* Display Inversion OFF */
#define TFT_ILI9341_DINVON              0x21   /* Display Inversion ON */
#define TFT_ILI9341_GAMMA               0x26   /* Gamma register */
#define TFT_ILI9341_DISPLAY_OFF         0x28   /* Display off register */
#define TFT_ILI9341_DISPLAY_ON          0x29   /* Display on register */
#define TFT_ILI9341_COLUMN_ADDR         0x2A   /* Colomn address register */
#define TFT_ILI9341_PAGE_ADDR           0x2B   /* Page address register */
#define TFT_ILI9341_GRAM                0x2C   /* GRAM register */
#define TFT_ILI9341_RGBSET              0x2D   /* Color SET */
#define TFT_ILI9341_RAMRD               0x2E   /* Memory Read */
#define TFT_ILI9341_PLTAR               0x30   /* Partial Area */
#define TFT_ILI9341_VSCRDEF             0x33   /* Vertical Scrolling Definition */
#define TFT_ILI9341_TEOFF               0x34   /* Tearing Effect Line OFF */
#define TFT_ILI9341_TEON                0x35   /* Tearing Effect Line ON */
#define TFT_ILI9341_MAC                 0x36   /* Memory Access Control register*/
#define TFT_ILI9341_VSCRSADD            0x37   /* Vertical Scrolling Start Address */
#define TFT_ILI9341_IDMOFF              0x38   /* Idle Mode OFF */
#define TFT_ILI9341_IDMON               0x39   /* Idle Mode ON */
#define TFT_ILI9341_PIXEL_FORMAT        0x3A   /* Pixel Format register */
#define TFT_ILI9341_WRITE_MEM_CONTINUE  0x3C   /* Write Memory Continue */
#define TFT_ILI9341_READ_MEM_CONTINUE   0x3E   /* Read Memory Continue */
#define TFT_ILI9341_SET_TEAR_SCANLINE   0x44   /* Set Tear Scanline */
#define TFT_ILI9341_GET_SCANLINE        0x45   /* Get Scanline */
#define TFT_ILI9341_WDB                 0x51   /* Write Brightness Display register */
#define TFT_ILI9341_RDDISBV             0x52   /* Read Display Brightness */
#define TFT_ILI9341_WCD                 0x53   /* Write Control Display register*/
#define TFT_ILI9341_RDCTRLD             0x54   /* Read CTRL Display */
#define TFT_ILI9341_WRCABC              0x55   /* Write Content Adaptive Brightness Control */
#define TFT_ILI9341_RDCABC              0x56   /* Read Content Adaptive Brightness Control */
#define TFT_ILI9341_WRITE_CABC          0x5E   /* Write CABC Minimum Brightness */
#define TFT_ILI9341_READ_CABC           0x5F   /* Read CABC Minimum Brightness */
#define TFT_ILI9341_READ_ID1            0xDA   /* Read ID1 */
#define TFT_ILI9341_READ_ID2            0xDB   /* Read ID2 */
#define TFT_ILI9341_READ_ID3            0xDC   /* Read ID3 */
/* Level 2 Commands */
#define TFT_ILI9341_RGB_INTERFACE       0xB0   /* RGB Interface Signal Control */
#define TFT_ILI9341_FRMCTR1             0xB1   /* Frame Rate Control (In Normal Mode) */
#define TFT_ILI9341_FRMCTR2             0xB2   /* Frame Rate Control (In Idle Mode) */
#define TFT_ILI9341_FRMCTR3             0xB3   /* Frame Rate Control (In Partial Mode) */
#define TFT_ILI9341_INVTR               0xB4   /* Display Inversion Control */
#define TFT_ILI9341_BPC                 0xB5   /* Blanking Porch Control register */
#define TFT_ILI9341_DFC                 0xB6   /* Display Function Control register */
#define TFT_ILI9341_ETMOD               0xB7   /* Entry Mode Set */
#define TFT_ILI9341_BACKLIGHT1          0xB8   /* Backlight Control 1 */
#define TFT_ILI9341_BACKLIGHT2          0xB9   /* Backlight Control 2 */
#define TFT_ILI9341_BACKLIGHT3          0xBA   /* Backlight Control 3 */
#define TFT_ILI9341_BACKLIGHT4          0xBB   /* Backlight Control 4 */
#define TFT_ILI9341_BACKLIGHT5          0xBC   /* Backlight Control 5 */
#define TFT_ILI9341_BACKLIGHT7          0xBE   /* Backlight Control 7 */
#define TFT_ILI9341_BACKLIGHT8          0xBF   /* Backlight Control 8 */
#define TFT_ILI9341_POWER1              0xC0   /* Power Control 1 register */
#define TFT_ILI9341_POWER2              0xC1   /* Power Control 2 register */
#define TFT_ILI9341_VCOM1               0xC5   /* VCOM Control 1 register */
#define TFT_ILI9341_VCOM2               0xC7   /* VCOM Control 2 register */
#define TFT_ILI9341_NVMWR               0xD0   /* NV Memory Write */
#define TFT_ILI9341_NVMPKEY             0xD1   /* NV Memory Protection Key */
#define TFT_ILI9341_RDNVM               0xD2   /* NV Memory Status Read */
#define TFT_ILI9341_READ_ID4            0xD3   /* Read ID4 */
#define TFT_ILI9341_PGAMMA              0xE0   /* Positive Gamma Correction register */
#define TFT_ILI9341_NGAMMA              0xE1   /* Negative Gamma Correction register */
#define TFT_ILI9341_DGAMCTRL1           0xE2   /* Digital Gamma Control 1 */
#define TFT_ILI9341_DGAMCTRL2           0xE3   /* Digital Gamma Control 2 */
#define TFT_ILI9341_INTERFACE           0xF6   /* Interface control register */

/* Extend register commands */
#define TFT_ILI9341_POWERA               0xCB   /* Power control A register */
#define TFT_ILI9341_POWERB               0xCF   /* Power control B register */
#define TFT_ILI9341_DTCA                 0xE8   /* Driver timing control A */
#define TFT_ILI9341_DTCB                 0xEA   /* Driver timing control B */
#define TFT_ILI9341_POWER_SEQ            0xED   /* Power on sequence register */
#define TFT_ILI9341_3GAMMA_EN            0xF2   /* 3 Gamma enable register */
#define TFT_ILI9341_PRC                  0xF7   /* Pump ratio control register */
//-------------------------------------------------------------------
#define SWAP(a,b) {int16_t t=a;a=b;b=t;}
//-------------------------------------------------------------------
void TFTDisplay_ILI9341_Initialization(uint16_t screenWidth, uint16_t screeHeight);
void TFTDisplay_ILI9341_FillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void TFTDisplay_ILI9341_FillScreen(uint16_t color);
void TFTDisplay_ILI9341_DrawPixel(int x, int y, uint16_t color);
void TFTDisplay_ILI9341_DrawLine(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t color);
void TFTDisplay_ILI9341_DrawClearRect(uint16_t x1, uint16_t y1,
											uint16_t x2, uint16_t y2, uint16_t color);
void TFTDisplay_ILI9341_DrawCircle(uint16_t x_Position, uint16_t y_Position, int radius, uint16_t color);
void TFTDisplay_ILI9341_SetTextColor(uint16_t color);
void TFTDisplay_ILI9341_SetBackColor(uint16_t color);
void TFTDisplay_ILI9341_SetFont(sFONT *pFonts);
void TFTDisplay_ILI9341_DrawChar(uint16_t x, uint16_t y, uint8_t c);
void TFTDisplay_ILI9341_String(uint16_t x,uint16_t y, char *str);
void TFTDisplay_ILI9341_SetRotation(uint8_t RotationSelectVal);
//-------------------------------------------------------------------
#endif /* __LL_SPI_ILI9341_H */
