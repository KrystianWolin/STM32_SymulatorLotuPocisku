#include "ll_spi_ili9341.h"
#include "font24.c"
#include "font20.c"
#include "font16.c"
#include "font12.c"
#include "font8.c"
//-------------------------------------------------------------------
uint16_t TFTDisplay_ILI9341_WIDTH = 0x00;
uint16_t TFTDisplay_ILI9341_HEIGHT = 0x00;
//-------------------------------------------------------------------
extern uint32_t DmaSpiCnt;
uint8_t DataBuffer[65536] = {0};
//-------------------------------------------------------------------
typedef struct
{
	uint16_t TextColor;
	uint16_t BackColor;
	sFONT *pFont;
}LCD_DrawPropTypeDef;
LCD_DrawPropTypeDef lcdprop;
//-------------------------------------------------------------------
static void tftDisplay_ILI9341_Init_PowerControl_A();
static void tftDisplay_ILI9341_Init_PowerControl_B();
static void tftDisplay_ILI9341_Init_DriverTimingControl_A();
static void tftDisplay_ILI9341_Init_DriverTimingControl_B();
static void tftDisplay_ILI9341_Init_PowerSequenceControl();
static void tftDisplay_ILI9341_Init_PumpRatioControl();
static void tftDisplay_ILI9341_Init_PowerControlVRH();
static void tftDisplay_ILI9341_Init_PowerControlSAP();
static void tftDisplay_ILI9341_Init_VCOMControl_1();
static void tftDisplay_ILI9341_Init_VCOMControl_2();
static void tftDisplay_ILI9341_Init_MemmoryAccessControl();
static void tftDisplay_ILI9341_Init_PixelFormatSet();
static void tftDisplay_ILI9341_Init_FrameRationControl();
static void tftDisplay_ILI9341_Init_DisplayFunctionControl();
static void tftDisplay_ILI9341_Init_GammaSet();
static void tftDisplay_ILI9341_Init_PositiveGammaCorection();
static void tftDisplay_ILI9341_Init_NegativeGammaCorection();
//-------------------------------------------------------------------
static void tftDisplay_ILI9341_SendCommand(uint8_t command);
static void tftDisplay_ILI9341_SendData(uint8_t data);
static void tftDisplay_ILI9341_SendSingleData(uint32_t* BufferPtr);
static void tftDisplay_ILI9341_WriteMultipleData(uint32_t* BufferPtr, uint32_t BufferSize, uint8_t DmaCount);
static void tftDisplay_ILI9341_Reset(void);
static void tftDisplay_ILI9341_FontsInitialization(void);
static void tftDisplay_ILI9341_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
//-------------------------------------------------------------------
void TFTDisplay_ILI9341_Initialization(uint16_t screenWidth, uint16_t screeHeight)
{
	CS_ACTIVE();
	tftDisplay_ILI9341_Reset();
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_SWRESET);

	LL_mDelay(1000);

	tftDisplay_ILI9341_Init_PowerControl_A();
	tftDisplay_ILI9341_Init_PowerControl_B();
	tftDisplay_ILI9341_Init_DriverTimingControl_A();
	tftDisplay_ILI9341_Init_DriverTimingControl_B();
	tftDisplay_ILI9341_Init_PowerSequenceControl();
	tftDisplay_ILI9341_Init_PumpRatioControl();
	tftDisplay_ILI9341_Init_PowerControlVRH();
	tftDisplay_ILI9341_Init_PowerControlSAP();
	tftDisplay_ILI9341_Init_VCOMControl_1();
	tftDisplay_ILI9341_Init_VCOMControl_2();
	tftDisplay_ILI9341_Init_MemmoryAccessControl();
	tftDisplay_ILI9341_Init_PixelFormatSet();
	tftDisplay_ILI9341_Init_FrameRationControl();
	tftDisplay_ILI9341_Init_DisplayFunctionControl();
	tftDisplay_ILI9341_Init_GammaSet();
	tftDisplay_ILI9341_Init_PositiveGammaCorection();
	tftDisplay_ILI9341_Init_NegativeGammaCorection();

	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_SLEEP_OUT);

	LL_mDelay(120);

	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_DISPLAY_ON);
	tftDisplay_ILI9341_SendData(TFT_ILI9341_ROTATION);
	TFTDisplay_ILI9341_WIDTH = screenWidth;
	TFTDisplay_ILI9341_HEIGHT = screeHeight;
	tftDisplay_ILI9341_FontsInitialization();
}
//-------------------------------------------------------------------
void TFTDisplay_ILI9341_FillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  uint32_t i = 0;
  uint32_t n = 0;
  uint8_t cnt = 0;
  uint32_t buf_size = 0;

  if((x1 >= TFTDisplay_ILI9341_WIDTH) || (y1 >= TFTDisplay_ILI9341_HEIGHT) ||
     (x2 >= TFTDisplay_ILI9341_WIDTH) || (y2 >= TFTDisplay_ILI9341_HEIGHT)) {
	  return;
  }
  if(x1>x2) {
	  SWAP(x1,x2);
  }
  if(y1>y2) {
	  SWAP(y1,y2);
  }
  tftDisplay_ILI9341_SetAddrWindow(x1, y1, x2, y2);
  n = (x2-x1+1)*(y2-y1+1)*2;
  if(n<=65535)
  {
    cnt = 1;
    buf_size = n;
  }
  else
  {
    cnt = n/2;
    buf_size = 2;
    for(i = 3; i < n/3; i++)
    {
      if(n%i == 0)
      {
        cnt = i;
        buf_size = n/i;
        break;
      }
    }
  }
  for(i = 0; i < buf_size/2; i++)
  {
	  DataBuffer[i*2] = color >> 8;
	  DataBuffer[i*2+1] = color & 0xFF;
  }

  tftDisplay_ILI9341_WriteMultipleData((uint32_t*)&DataBuffer[0], buf_size, cnt);
}
//-------------------------------------------------------------------
void TFTDisplay_ILI9341_FillScreen(uint16_t color)
{
	TFTDisplay_ILI9341_FillRect(0, 0, TFTDisplay_ILI9341_WIDTH-1, TFTDisplay_ILI9341_HEIGHT-1, color);
}
//-------------------------------------------------------------------
void TFTDisplay_ILI9341_DrawPixel(int x, int y, uint16_t color)
{
	if((x<0)||(y<0)||(x>=TFTDisplay_ILI9341_WIDTH)||(y>=TFTDisplay_ILI9341_HEIGHT))
	{
		return;
	}

	tftDisplay_ILI9341_SetAddrWindow(x,y,x,y);
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_GRAM);
	DataBuffer[0] = color>>8;
	DataBuffer[1] = color & 0xFF;


	tftDisplay_ILI9341_WriteMultipleData((uint32_t*)&DataBuffer[0], 2, 1);
}
//------------------------------------------------------------------
void TFTDisplay_ILI9341_DrawLine(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t color)
{
    int steep = abs(y_end-y_start)>abs(x_end-x_start);
    int dx=0;
    int dy=0;
    int err=0;
    int ystep=0;

    if (steep)
    {
    	SWAP(x_start,y_start);
    	SWAP(x_end,y_end);
    }

    if(x_start>x_end)
    {
    	SWAP(x_start,x_end);
    	SWAP(y_start,y_end);
    }

    dx=x_end-x_start;
    dy=abs(y_end-y_start);
    err=dx/2;

    if(y_start<y_end)
    {
        ystep = 1;
    }
    else
    {
        ystep = -1;
    }
    for (;x_start<=x_end;x_start++)
    {
       if (steep)
       {
    	   TFTDisplay_ILI9341_DrawPixel(y_start,x_start,color);
       }
       else
       {
    	   TFTDisplay_ILI9341_DrawPixel(x_start,y_start,color);
       }
       err-=dy;
       if (err<0)
       {
        y_start += ystep;
        err+=dx;
       }
    }
}
//-------------------------------------------------------------------
void TFTDisplay_ILI9341_DrawClearRect(uint16_t start_x, uint16_t start_y,
        								uint16_t end_x, uint16_t end_y, uint16_t color)
{
	TFTDisplay_ILI9341_DrawLine(start_x,start_y,end_x,start_y,color);
    TFTDisplay_ILI9341_DrawLine(end_x, start_y, end_x, end_y, color);
    TFTDisplay_ILI9341_DrawLine(start_x, start_y, start_x, end_y, color);
    TFTDisplay_ILI9341_DrawLine(start_x, end_y, end_x, end_y, color);
}
//-------------------------------------------------------------------
void TFTDisplay_ILI9341_DrawCircle(uint16_t x_Position, uint16_t y_Position, int radius, uint16_t color)
{
	int f = 1-radius;
	int ddF_x=1;
	int ddF_y=-2*radius;
	int x = 0;
	int y = radius;

	TFTDisplay_ILI9341_DrawPixel(x_Position,y_Position+radius,color);
	TFTDisplay_ILI9341_DrawPixel(x_Position,y_Position-radius,color);
	TFTDisplay_ILI9341_DrawPixel(x_Position+radius,y_Position,color);
	TFTDisplay_ILI9341_DrawPixel(x_Position-radius,y_Position,color);
	while (x<y)
	{
		if (f>=0)
		{
			y--;
			ddF_y+=2;
			f+=ddF_y;
	    }

		x++;
	    ddF_x+=2;
	    f+=ddF_x;
	    TFTDisplay_ILI9341_DrawPixel(x_Position+x,y_Position+y,color);
	    TFTDisplay_ILI9341_DrawPixel(x_Position-x,y_Position+y,color);
	    TFTDisplay_ILI9341_DrawPixel(x_Position+x,y_Position-y,color);
	    TFTDisplay_ILI9341_DrawPixel(x_Position-x,y_Position-y,color);
	    TFTDisplay_ILI9341_DrawPixel(x_Position+y,y_Position+x,color);
	    TFTDisplay_ILI9341_DrawPixel(x_Position-y,y_Position+x,color);
	    TFTDisplay_ILI9341_DrawPixel(x_Position+y,y_Position-x,color);
	    TFTDisplay_ILI9341_DrawPixel(x_Position-y,y_Position-x,color);
	}
}
//-------------------------------------------------------------------
void TFTDisplay_ILI9341_SetTextColor(uint16_t color)
{
  lcdprop.TextColor=color;
}
//------------------------------------
void TFTDisplay_ILI9341_SetBackColor(uint16_t color)
{
  lcdprop.BackColor=color;
}
//------------------------------------
void TFTDisplay_ILI9341_SetFont(sFONT *pFonts)
{
  lcdprop.pFont=pFonts;
}
//-------------------------------------------------------------------
void TFTDisplay_ILI9341_DrawChar(uint16_t x, uint16_t y, uint8_t c)
{
  uint32_t i = 0, j = 0;
  uint16_t height, width;
  uint16_t y_cur = y;
  uint8_t offset;
  uint8_t *c_t;
  uint8_t *pchar;
  uint32_t line=0;
  height = lcdprop.pFont->Height;
  width  = lcdprop.pFont->Width;
  offset = 8 *((width + 7)/8) -  width ;
  c_t = (uint8_t*) &(lcdprop.pFont->table[(c-' ') * lcdprop.pFont->Height * ((lcdprop.pFont->Width + 7) / 8)]);
  for(i = 0; i < height; i++)
  {
    pchar = ((uint8_t *)c_t + (width + 7)/8 * i);
    switch(((width + 7)/8))
    {
      case 1:
          line =  pchar[0];      
          break;
      case 2:
          line =  (pchar[0]<< 8) | pchar[1];
          break;
      case 3:
      default:
        line =  (pchar[0]<< 16) | (pchar[1]<< 8) | pchar[2];      
        break;
    }
    for (j = 0; j < width; j++)
    {
      if(line & (1 << (width- j + offset- 1))) 
      {
    	DataBuffer[(i*width + j) * 2] = lcdprop.TextColor >> 8;
    	DataBuffer[(i*width + j)*2+1] = lcdprop.TextColor & 0xFF;
      }
      else
      {
    	DataBuffer[(i*width + j)*2] = lcdprop.BackColor >> 8;
        DataBuffer[(i*width + j)*2+1] = lcdprop.BackColor & 0xFF;
      } 
    }
    y_cur++;
  }

  DmaSpiCnt = 1;
  tftDisplay_ILI9341_SetAddrWindow(x, y, x+width-1, y+height-1);
  tftDisplay_ILI9341_WriteMultipleData((uint32_t*)&DataBuffer[0], width * height * 2, DmaSpiCnt);
}
//-------------------------------------------------------------------
void TFTDisplay_ILI9341_String(uint16_t x, uint16_t y, char *str)
{
  while(*str)
  {
	  TFTDisplay_ILI9341_DrawChar(x,y,str[0]);
      x+=lcdprop.pFont->Width;
      (void)*str++;
  }
}
//-------------------------------------------------------------------
void TFTDisplay_ILI9341_SetRotation(uint8_t RotationSelectVal)
{
  tftDisplay_ILI9341_SendCommand(0x36);
  switch(RotationSelectVal)
  {
    case 0:
      DataBuffer[0] = ORIENTATION_PORTRAIT;
      tftDisplay_ILI9341_SendSingleData((uint32_t*)&DataBuffer[0]);
      TFTDisplay_ILI9341_WIDTH = 240;
      TFTDisplay_ILI9341_HEIGHT = 320;
      break;
    case 1:
      DataBuffer[0] = ORIENTATION_LANDSCAPE;
      tftDisplay_ILI9341_SendSingleData((uint32_t*)&DataBuffer[0]);
      TFTDisplay_ILI9341_WIDTH = 320;
      TFTDisplay_ILI9341_HEIGHT = 240;
      break;
    case 2:
      DataBuffer[0] = ORIENTATION_PORTRAIT_MIRROR;
      tftDisplay_ILI9341_SendSingleData((uint32_t*)&DataBuffer[0]);
      TFTDisplay_ILI9341_WIDTH = 240;
      TFTDisplay_ILI9341_HEIGHT = 320;
      break;
    case 3:
      DataBuffer[0] = ORIENTATION_LANDSCAPE_MIRROR;
      tftDisplay_ILI9341_SendSingleData((uint32_t*)&DataBuffer[0]);
      TFTDisplay_ILI9341_WIDTH = 320;
      TFTDisplay_ILI9341_HEIGHT = 240;
      break;
  }
}
//-------------------------------------------------------------------
//-------------------------------------------------------------------
static void tftDisplay_ILI9341_SendCommand(uint8_t command)
{
	CS_ACTIVE();
	DC_SET_SEND_COMMAND();
	DmaSpiCnt = 1;
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_4, 1);
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_4, (uint32_t)&command,
			LL_SPI_DMA_GetRegAddr(SPI5), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_4));
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_4);

	while(((DMA2_Stream4->CR & DMA_SxCR_EN) != 0) || ((DMA2->HISR & DMA_HISR_TCIF4) != 0)) { }
}
static void tftDisplay_ILI9341_SendData(uint8_t data)
{
//	  CS_ACTIVE();
//	  DC_SET_SEND_DATA();
//
//	  DmaSpiCnt = 1;
//	  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
//	  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_4, 1);
//	  LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_4,
//			  (uint32_t)&data,
//			  LL_SPI_DMA_GetRegAddr(SPI5),
//			  LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_4));
//	  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_4);
//
//	  while(((DMA2_Stream4->CR & DMA_SxCR_EN) != 0) || ((DMA2->HISR & DMA_HISR_TCIF4) != 0)) { }
//	  //CS_DESELECT();

	CS_ACTIVE();
	DC_SET_SEND_DATA();
    while(!LL_SPI_IsActiveFlag_TXE(SPI5)) {}
	LL_SPI_TransmitData8 (SPI5, data);
	while(!LL_SPI_IsActiveFlag_RXNE(SPI5)) {}
	LL_SPI_ReceiveData8(SPI5);
	CS_DESELECT();
}

static void tftDisplay_ILI9341_SendSingleData(uint32_t* BufferPtr)
{
	CS_ACTIVE();
	DC_SET_SEND_DATA();
	DmaSpiCnt = 1;
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_4, 1);
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_4,
			(uint32_t)BufferPtr,
			LL_SPI_DMA_GetRegAddr(SPI5),
			LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_4));
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_4);

	while(((DMA2_Stream4->CR & DMA_SxCR_EN) != 0) || ((DMA2->HISR & DMA_HISR_TCIF4) != 0)) { }
}

static void tftDisplay_ILI9341_WriteMultipleData(uint32_t* BufferPtr, uint32_t BufferSize, uint8_t DmaCount)
{
	CS_ACTIVE();
	DC_SET_SEND_DATA();
	DmaSpiCnt = DmaCount;
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_4, BufferSize);
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_4,
			(uint32_t)BufferPtr,
			LL_SPI_DMA_GetRegAddr(SPI5),
			LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_4));
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_4);

	while(((DMA2_Stream4->CR & DMA_SxCR_EN) != 0) || ((DMA2->HISR & DMA_HISR_TCIF4) != 0)) { }
}
static void tftDisplay_ILI9341_Reset(void)
{
	RESET_ACTIVE();
	LL_mDelay(5);
	RESET_IDLE();
}
static void tftDisplay_ILI9341_FontsInitialization(void)
{
  Font8.Height = 8;
  Font8.Width = 5;
  Font12.Height = 12;
  Font12.Width = 7;
  Font16.Height = 16;
  Font16.Width = 11;
  Font20.Height = 20;
  Font20.Width = 14;
  Font24.Height = 24;
  Font24.Width = 17;
  lcdprop.BackColor=TFT_COLOR_ILI9341_BLACK;
  lcdprop.TextColor=TFT_COLOR_ILI9341_GREEN;
  lcdprop.pFont=&Font16;
}
static void tftDisplay_ILI9341_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  tftDisplay_ILI9341_SendCommand(TFT_ILI9341_COLUMN_ADDR);
  DataBuffer[0] = (x0 >> 8) & 0xFF;
  DataBuffer[1] = x0 & 0xFF;
  DataBuffer[2] = (x1 >> 8) & 0xFF;
  DataBuffer[3] = x1 & 0xFF;
  tftDisplay_ILI9341_WriteMultipleData((uint32_t*)&DataBuffer[0], 4, 1);

  tftDisplay_ILI9341_SendCommand(TFT_ILI9341_PAGE_ADDR);
  DataBuffer[0] = (y0 >> 8) & 0xFF;
  DataBuffer[1] = y0 & 0xFF;
  DataBuffer[2] = (y1 >> 8) & 0xFF;
  DataBuffer[3] = y1 & 0xFF;
  tftDisplay_ILI9341_WriteMultipleData((uint32_t*)&DataBuffer[0], 4, 1);

  tftDisplay_ILI9341_SendCommand(TFT_ILI9341_GRAM);
}
//------------------------------------------------------------------
/* Initialization static functions */
static void tftDisplay_ILI9341_Init_PowerControl_A()
{
	DataBuffer[0] = 0x39;
	DataBuffer[1] = 0x2C;
	DataBuffer[2] = 0x00;
	DataBuffer[3] = 0x34;
	DataBuffer[4] = 0x02;
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_POWERA);
	tftDisplay_ILI9341_WriteMultipleData((uint32_t*)&DataBuffer[0], 5, 1);
}

static void tftDisplay_ILI9341_Init_PowerControl_B()
{
	DataBuffer[0] = 0x00;
	DataBuffer[1] = 0xC1;
	DataBuffer[2] = 0x30;
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_POWERB);
	tftDisplay_ILI9341_WriteMultipleData((uint32_t*)&DataBuffer[0], 3, 1);
}

static void tftDisplay_ILI9341_Init_DriverTimingControl_A()
{
	DataBuffer[0] = 0x85;
	DataBuffer[1] = 0x00;
	DataBuffer[2] = 0x78;
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_DTCB);
	tftDisplay_ILI9341_WriteMultipleData((uint32_t*)&DataBuffer[0], 3, 1);
}

static void tftDisplay_ILI9341_Init_DriverTimingControl_B()
{
	DataBuffer[0] = 0x00;
	DataBuffer[1] = 0x00;
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_DTCB);
	tftDisplay_ILI9341_WriteMultipleData((uint32_t*)&DataBuffer[0], 2, 1);
}

static void tftDisplay_ILI9341_Init_PowerSequenceControl()
{
	DataBuffer[0] = 0x64;
	DataBuffer[1] = 0x03;
	DataBuffer[2] = 0x12;
	DataBuffer[3] = 0x81;
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_POWER_SEQ);
	tftDisplay_ILI9341_WriteMultipleData((uint32_t*)&DataBuffer[0], 4, 1);
}

static void tftDisplay_ILI9341_Init_PumpRatioControl()
{
	DataBuffer[0] = 0x20;
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_PRC);
	tftDisplay_ILI9341_SendSingleData((uint32_t*)&DataBuffer[0]);
}

static void tftDisplay_ILI9341_Init_PowerControlVRH()
{
	DataBuffer[0] = 0x10;
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_POWER1);
	tftDisplay_ILI9341_SendSingleData((uint32_t*)&DataBuffer[0]);
}

static void tftDisplay_ILI9341_Init_PowerControlSAP()
{
	DataBuffer[0] = 0x10;
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_POWER2);
	tftDisplay_ILI9341_SendSingleData((uint32_t*)&DataBuffer[0]);
}

static void tftDisplay_ILI9341_Init_VCOMControl_1()
{
	DataBuffer[0] = 0x3E;
	DataBuffer[1] = 0x28;
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_VCOM1);
	tftDisplay_ILI9341_WriteMultipleData((uint32_t*)&DataBuffer[0], 2, 1);
}

static void tftDisplay_ILI9341_Init_VCOMControl_2()
{
	DataBuffer[0] = 0x86;
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_VCOM2);
	tftDisplay_ILI9341_SendSingleData((uint32_t*)&DataBuffer[0]);
}

static void tftDisplay_ILI9341_Init_MemmoryAccessControl()
{
	DataBuffer[0] = 0x48;
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_MAC);
	tftDisplay_ILI9341_SendSingleData((uint32_t*)&DataBuffer[0]);
}

static void tftDisplay_ILI9341_Init_PixelFormatSet()
{
	DataBuffer[0] = 0x55;
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_PIXEL_FORMAT);
	tftDisplay_ILI9341_SendSingleData((uint32_t*)&DataBuffer[0]);
}

static void tftDisplay_ILI9341_Init_FrameRationControl()
{
	DataBuffer[0] = 0x00;
	DataBuffer[1] = 0x18;
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_FRMCTR1);
	tftDisplay_ILI9341_WriteMultipleData((uint32_t*)&DataBuffer[0], 2, 1);
}

static void tftDisplay_ILI9341_Init_DisplayFunctionControl()
{
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_DFC);
	tftDisplay_ILI9341_SendData(0x08);
	tftDisplay_ILI9341_SendData(0x82);
	tftDisplay_ILI9341_SendData(0x27);
}

static void tftDisplay_ILI9341_Init_GammaSet()
{
	DataBuffer[0] = 0x01;
	tftDisplay_ILI9341_SendCommand(TFT_ILI9341_GAMMA);
	tftDisplay_ILI9341_SendSingleData((uint32_t*)&DataBuffer[0]);
}

static void tftDisplay_ILI9341_Init_PositiveGammaCorection()
{
	  DataBuffer[0] = 0x0F;
	  DataBuffer[1] = 0x31;
	  DataBuffer[2] = 0x2B;
	  DataBuffer[3] = 0x0C;
	  DataBuffer[4] = 0x0E;
	  DataBuffer[5] = 0x08;
	  DataBuffer[6] = 0x4E;
	  DataBuffer[7] = 0xF1;
	  DataBuffer[8] = 0x37;
	  DataBuffer[9] = 0x07;
	  DataBuffer[10] = 0x10;
	  DataBuffer[11] = 0x03;
	  DataBuffer[12] = 0x0E;
	  DataBuffer[13] = 0x09;
	  DataBuffer[14] = 0x00;
	  tftDisplay_ILI9341_SendCommand(TFT_ILI9341_PGAMMA);
	  tftDisplay_ILI9341_WriteMultipleData((uint32_t*)&DataBuffer[0], 15, 1);
}

static void tftDisplay_ILI9341_Init_NegativeGammaCorection()
{
	  DataBuffer[0] = 0x00;
	  DataBuffer[1] = 0x0E;
	  DataBuffer[2] = 0x14;
	  DataBuffer[3] = 0x03;
	  DataBuffer[4] = 0x11;
	  DataBuffer[5] = 0x07;
	  DataBuffer[6] = 0x31;
	  DataBuffer[7] = 0xC1;
	  DataBuffer[8] = 0x48;
	  DataBuffer[9] = 0x08;
	  DataBuffer[10] = 0x0F;
	  DataBuffer[11] = 0x0C;
	  DataBuffer[12] = 0x31;
	  DataBuffer[13] = 0x36;
	  DataBuffer[14] = 0x0F;
	  tftDisplay_ILI9341_SendCommand(TFT_ILI9341_NGAMMA);
	  tftDisplay_ILI9341_WriteMultipleData((uint32_t*)&DataBuffer[0], 15, 1);
}
