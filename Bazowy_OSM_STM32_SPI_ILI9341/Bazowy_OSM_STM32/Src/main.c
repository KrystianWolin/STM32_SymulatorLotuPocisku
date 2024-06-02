/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ll_spi_ili9341.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* Definitions for LCD_Task */
osThreadId_t LCD_TaskHandle;
const osThreadAttr_t LCD_Task_attributes = {
  .name = "LCD_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LED1_task */
osThreadId_t LED1_taskHandle;
const osThreadAttr_t LED1_task_attributes = {
  .name = "LED1_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LED2_tack */
osThreadId_t LED2_tackHandle;
const osThreadAttr_t LED2_tack_attributes = {
  .name = "LED2_tack",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BinarySem01 */
osSemaphoreId_t BinarySem01Handle;
const osSemaphoreAttr_t BinarySem01_attributes = {
  .name = "BinarySem01"
};
/* USER CODE BEGIN PV */
uint8_t DmaSpiCnt=1;
/*
int x = 120, y = 160; // Skladowe x,y polozenia kwadratu
int x_old = 120, y_old = 160;
int dx = 1, dy = 1; // Skladowe wektora predkosci dx, dy
*/

unsigned char *ptr;
unsigned char napis[] = {"Press the blue button..."};
int dl_n = 24;
unsigned int nx = 0;
unsigned int ny = 280;
unsigned int  key;

//
unsigned int wysEkranu = 240;
unsigned int szeEkranu = 320;
unsigned int offsetPodloza = 24;
double mX = 240;
double mY = 0.002;

int miejsceLadowania = 0;

//unsigned int czestOdswAnimacji = 500;//czestotliwosc odswierzania animacji lotu
//unsigned int czasAnimacji = (int)(1000 / czestOdswAnimacji);//wyliczenie czasu animacji
unsigned int czasAnimacji = 2;
unsigned int preScaleZapisany = 10000;//moment preskalera licznika uzytkownika w ktorym zakonczyla sie analiza
bool koniecAnalizy = false; //ustawiana i restartowana w celu wyznaczenia momentu analizy
bool koniecRysowania = false; //ustawiana i restartowana w celu wyznaczenia momentu rysowania

unsigned int prepreScale = 0;
int Tim = 0;                // Licznik uzytkownika
unsigned int preScale = 0;  // Preskaler licznika uzytkownika

enum Tryb{
    RYSOWANIE,
    WYKRESY
};

enum Tryb trybPracy = RYSOWANIE;

int daneZadana[105];
int daneWynik[105];
int daneUchyb[105];
//

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RNG_Init(void);
static void MX_SPI5_Init(void);
void StartLCDTask(void *argument);
void StartTaskLED1(void *argument);
void StartTaskLED2(void *argument);

/* USER CODE BEGIN PFP */
//void move_square_C(void);

void setKat(double kat);
double getKat();
void setCelReg(double cel);
double getCelReg();
void setPredkosc(unsigned int predkosc);
unsigned int getPredkosc();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
void move_square_C(){
//Aktualizacja po│ozenia na podstawie wektora predkosci
		x+=dx;
		y+=dy;
//Ograniczenie obszaru poruszania kwadratem w obrebie LCD
		if (x <= 1 || x >= 240 - 34)
			dx = -dx;
		if (y <= 1 || y >= 320 - 34)
			dy = -dy;
}
*/

//obiekt regulacji
struct ObiektRegulacji {
    double katArmaty;
    double cel;
    unsigned int predkosc;
};

struct ObiektRegulacji objReg;

    void setKat(double kat) {
        if (kat < 0 || kat>90) {}
        else objReg.katArmaty = kat;
    }

    double getKat() {
        return objReg.katArmaty;
    }

    void setCelReg(double cel) {
        if (cel <= 0 || cel >= 240) {}
        else objReg.cel = cel;
    }

    double getCelReg() {
        return objReg.cel;
    }

    void setPredkosc(unsigned int predkosc) {
        if (predkosc <= 0) {}
        else objReg.predkosc = predkosc;
    }

    unsigned int getPredkosc() {
        return objReg.predkosc;
    }
//

//
    double stopnieNaRad(double stopnie)
    {
        return (stopnie * M_PI) / 180;
    }

    double radNaStopnie(double rad)
    {
        return (rad * 180) / M_PI;
    }

    double pozNaRad(double x, int offset, double v) //ta funkcja nie ma sensu
    {
        double stopnie = asin(((128 - 2 * offset) / mY + (9.8 * x * x) / 2) / (v * x * mX));
        return stopnie;
    }


    void AnalizaIRegulacja()
    {
        /*
        unsigned char Text[] = "AnalizaIRegulacja___";

            for (int j = 0; j < rozmiar; j++)
            {
                Komentarze[j] = Text[j];
            }

        PrintText(textEkran, Komentarze, 20, 0, 0);
*/

        double stopnie = getKat();
/*
        unsigned char Text1[] = "Stopnie_przedReg____";

            for (int j = 0; j < rozmiar; j++)
            {
                Komentarze[j] = Text1[j];
            }

            Komentarze[17] = ((int)stopnie / 100 - 10 * ((int)stopnie / 1000) + '0');
            Komentarze[18] = (int)stopnie / 10 - 10 * ((int)stopnie / 100) + '0';
            Komentarze[19] = (int)stopnie % 10 + '0';

        PrintText(textEkran, Komentarze, 20, 0, 1);
*/

        double uchyb = getCelReg() - miejsceLadowania;

        //DodajDaneUchyb(uchyb);
/*
        unsigned char Text3[] = "Uchyb_______________";

            for (int j = 0; j < rozmiar; j++)
            {
                Komentarze[j] = Text3[j];
            }

            if(uchyb<0) Komentarze[16] = '-';
            Komentarze[17] = (abs(uchyb) / 100 - 10 * (abs(uchyb) / 1000) + '0');
            Komentarze[18] = abs(uchyb) / 10 - 10 * (abs(uchyb) / 100) + '0';
            Komentarze[19] = abs(uchyb) % 10 + '0';

        PrintText(textEkran, Komentarze, 20, 20, 5);
*/
    /*
        regPID.setInput(uchyb);

        regPID.Calculate();

        double zadana = regPID.getOutput();
    */
        //cout<<"\nZadane miejsce ladowania: "<<zadana;

    //    stopnie = (int)radNaStopnie(pozNaRad(zadana, offsetPodloza, objReg.getPredkosc()));
    /*    if(uchyb<0) stopnie-=10; //proporcje 0-90 na 0-szerokosc ekranu
        else if(uchyb>0) stopnie+=10;
        else stopnie=stopnie;
    */

        //regulacja kata wystrzalu armaty
        stopnie+=uchyb/10;

        //cout<<" Stopnie; "<<stopnie;

        setKat(stopnie);

/*
        unsigned char Text2[] = "Stopnie_poReg_______";

            for (int j = 0; j < rozmiar; j++)
            {
                Komentarze[j] = Text2[j];
            }

            Komentarze[17] = ((int)stopnie / 100 - 10 * ((int)stopnie / 1000) + '0');
            Komentarze[18] = (int)stopnie / 10 - 10 * ((int)stopnie / 100) + '0';
            Komentarze[19] = (int)stopnie % 10 + '0';

        PrintText(textEkran, Komentarze, 20, 0, 2);
*/
        koniecAnalizy = true;
    }

    unsigned int t = 0;
    unsigned int x = 0;
    unsigned int y = 0;

        int ileRysowanie=0;

        void RysowanieTrajektorii()
        {
/*
            unsigned char Text[] = "RysowanieTrajektorii";

                for (int j = 0; j < rozmiar; j++)
                {
                    Komentarze[j] = Text[j];
                }

            PrintText(textEkran, Komentarze, 20, 0, 0);
*/

            //rysowanie tla

            int dlugoscA = 10;
            int szerokoscA = 10;


            //z kata obliczyc punkt (x,y) konca armaty

            int xA = dlugoscA * cos(stopnieNaRad(getKat()));
            //cout<<xA;
            int yA = dlugoscA * sin(stopnieNaRad(getKat()));
            //cout<<yA;
            //narysowac linie laczaca punkt poczatkowy z (x,y) i linie wyzej, nizej
            for (int dlugosc = 0; dlugosc < dlugoscA; dlugosc++)
            {
                for (int szerokosc = -szerokoscA / 2; szerokosc < szerokoscA / 2; szerokosc++)
                {
                    if(trybPracy == 0)
                        //SetPixel(ekran, (5 + (xA * dlugosc / dlugoscA) + szerokosc), (wysEkranu - (5 + offsetPodloza + (yA * dlugosc / dlugoscA) + szerokosc)));
                        TFTDisplay_ILI9341_DrawPixel((wysEkranu - (5 + offsetPodloza + (yA * dlugosc / dlugoscA) + szerokosc)), (5 + (xA * dlugosc / dlugoscA) + szerokosc), TFT_COLOR_ILI9341_BLACK);
                }
            }

            //wyliczanie pozycji kuli dla danej chwili
            t = (int)(preScale - preScaleZapisany)/5; // /25 = korekta zeby trajektoria rysowala sie wolniej

            x = (unsigned int)(0.38 * getPredkosc() * t * cos(stopnieNaRad(getKat())));

            y = (unsigned int)(mY * (getPredkosc() * t * mX * sin(stopnieNaRad(getKat())) - (9.8 * t * t / 2)) + offsetPodloza);


            if (abs((int)(offsetPodloza - y)) < 3)
            {
                miejsceLadowania = x;
/*
                unsigned char Text[] = "Nowe ladowanie = ___";

                    for (int j = 0; j < rozmiar; j++)
                    {
                        Komentarze[j] = Text[j];
                    }

                    Komentarze[17] = (miejsceLadowania / 100 - 10 * (miejsceLadowania / 1000) + '0');
                    Komentarze[18] = miejsceLadowania / 10 - 10 * (miejsceLadowania / 100) + '0';
                    Komentarze[19] = miejsceLadowania % 10 + '0';

                PrintText(textEkran, Komentarze, 20, 0, 4);
*/
            }

            if (y >= offsetPodloza && y < wysEkranu && x >= 0 && x < szeEkranu) //XD
            {
                if(trybPracy == 0)
                    //SetPixel(ekran, x, (wysEkranu - y));
                //    TFTDisplay_ILI9341_DrawPixel((wysEkranu - y), x, TFT_COLOR_ILI9341_RED);
                    TFTDisplay_ILI9341_DrawCircle((wysEkranu - y), x, 5, TFT_COLOR_ILI9341_RED);
            }
            else if (y >= wysEkranu || x >= szeEkranu)
            {
            }
            else
            {
                koniecRysowania = true;
                preScaleZapisany = preScale;
/*
                unsigned char Text[] = "Koniec rysowania____";

                    for (int j = 0; j < rozmiar; j++)
                    {
                        Komentarze[j] = Text[j];
                    }

                PrintText(textEkran, Komentarze, 20, 0, 0);
*/
                //DodajDaneWynik(miejsceLadowania);
            }
        }

        void Wykresy()
        {
            TFTDisplay_ILI9341_FillScreen(TFT_COLOR_ILI9341_WHITE);

            //240x128
            for(unsigned int x=1; x<szeEkranu; x++)
            {
                for(unsigned int y=1; y<wysEkranu; y++)
                {
                    //wykres lewo (wartosc zadana i uzyskana)
                    if(x==5 && y>=wysEkranu-75 && y<wysEkranu-5)
                        //SetPixel(ekran, x, y);
                        TFTDisplay_ILI9341_DrawPixel(y, x, TFT_COLOR_ILI9341_BLUE);
                    if(y==wysEkranu-5 && x>5 && x<110)
                        //SetPixel(ekran, x, y);
                        TFTDisplay_ILI9341_DrawPixel(y, x, TFT_COLOR_ILI9341_BLUE);

                    //wykres prawo (uchyb)
                    if(x==5+120 && y>=wysEkranu-75 && y<wysEkranu-5)
                        //SetPixel(ekran, x, y);
                        TFTDisplay_ILI9341_DrawPixel(y, x, TFT_COLOR_ILI9341_BLUE);
                    if(y==wysEkranu-40 && x>5+120 && x<110+120)
                        //SetPixel(ekran, x, y);
                        TFTDisplay_ILI9341_DrawPixel(y, x, TFT_COLOR_ILI9341_BLUE);
                }
            }
            for(int i=0; i<105; i++)
            {
                //wykresZadana
                //SetPixel(ekran, (i+5), (int)(wysEkranu-(daneZadana[i]/240.0*70+5)));
                TFTDisplay_ILI9341_DrawPixel((int)(wysEkranu-(daneZadana[i]/240.0*70+5)), (i+5), TFT_COLOR_ILI9341_BLUE);

                //wykresWynik
                //SetPixel(ekran, (i+5), (int)(wysEkranu-(daneWynik[i]/240.0*70+5)));
                TFTDisplay_ILI9341_DrawPixel((int)(wysEkranu-(daneWynik[i]/240.0*70+5)), (i+5), TFT_COLOR_ILI9341_BLUE);

                //wykresUchyb
                //SetPixel(ekran, (i+5+120), (int)(wysEkranu-(daneUchyb[i]/240.0*70+40)));
                TFTDisplay_ILI9341_DrawPixel((int)(wysEkranu-(daneUchyb[i]/240.0*70+40)), (i+5+120), TFT_COLOR_ILI9341_BLUE);
            }
        }


    void DodajDaneZadana(int liczba)
    {
        for(int i=0; i<104; i++)
        {
            daneZadana[i]=daneZadana[i+1];
        }
        daneZadana[104]=liczba;
    }

    void DodajDaneWynik(int liczba)
    {
        for(int i=0; i<104; i++)
            {
                daneWynik[i]=daneWynik[i+1];
            }
        daneWynik[104]=liczba;
    }

    void DodajDaneUchyb(int liczba)
    {
        for(int i=0; i<104; i++)
            {
                daneUchyb[i]=daneUchyb[i+1];
            }
        daneUchyb[104]=liczba;
    }
//

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//ptr = napis;

    for(int i=0; i<105;i++)
    {
        daneZadana[i]=0;
        daneWynik[i]=0;
        daneUchyb[i]=0;
    }

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */
    objReg.katArmaty = 30;
    objReg.cel = 100;
    objReg.predkosc = 5;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RNG_Init();
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
  LL_DMA_ClearFlag_TC4(DMA2);
  LL_DMA_ClearFlag_TE4(DMA2);
  LL_SPI_EnableDMAReq_TX(SPI5);
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_4);
  LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_4);
  LL_SPI_Enable(SPI5);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BinarySem01 */
  BinarySem01Handle = osSemaphoreNew(1, 1, &BinarySem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LCD_Task */
  LCD_TaskHandle = osThreadNew(StartLCDTask, NULL, &LCD_Task_attributes);

  /* creation of LED1_task */
  //LED1_taskHandle = osThreadNew(StartTaskLED1, NULL, &LED1_task_attributes);

  /* creation of LED2_tack */
  //LED2_tackHandle = osThreadNew(StartTaskLED2, NULL, &LED2_tack_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_PWR_EnableOverDriveMode();
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 180, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 180, LL_RCC_PLLQ_DIV_8);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(180000000);
  LL_SetSystemCoreClock(180000000);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_RNG);

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  LL_RNG_Enable(RNG);
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI5);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  /**SPI5 GPIO Configuration
  PF7   ------> SPI5_SCK
  PF8   ------> SPI5_MISO
  PF9   ------> SPI5_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7|LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* SPI5 DMA Init */

  /* SPI5_TX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_4, LL_DMA_CHANNEL_2);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_4, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_4, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_4, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_4);

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI5, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI5, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA interrupt init */
  /* DMA2_Stream4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOG);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_2);

  /**/
  LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_12|LL_GPIO_PIN_13);

  /**/
  LL_GPIO_ResetOutputPin(GPIOG, LL_GPIO_PIN_13|LL_GPIO_PIN_14);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12|LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void DMA1_Stream4_TransferComplete(void)
{
  LL_DMA_ClearFlag_TC4(DMA2);
  DmaSpiCnt--;

  if(DmaSpiCnt == 0)
  {
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
    DmaSpiCnt=1;
    CS_DESELECT();
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLCDTask */
/**
  * @brief  Function implementing the LCD_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLCDTask */
void StartLCDTask(void *argument)
{
  /* USER CODE BEGIN 5 */

	  TFTDisplay_ILI9341_Initialization(240, 320); //nie usuwac INITIALITATION
	  TFTDisplay_ILI9341_SetRotation(0);

	  //TFTDisplay_ILI9341_FillRect(5,5,35,35,TFT_COLOR_ILI9341_WHITE );
	  /*
	  TFTDisplay_ILI9341_FillScreen(TFT_COLOR_ILI9341_GREEN);
	  osDelay(1000);
	  TFTDisplay_ILI9341_FillScreen(TFT_COLOR_ILI9341_RED);
	  osDelay(1000);
	  TFTDisplay_ILI9341_FillScreen(TFT_COLOR_ILI9341_BLUE);
	  osDelay(1000);
	  TFTDisplay_ILI9341_FillRect(5,5,35,35,TFT_COLOR_ILI9341_WHITE );
	  TFTDisplay_ILI9341_DrawClearRect(40,40, 80, 80, TFT_COLOR_ILI9341_RED);
	  TFTDisplay_ILI9341_DrawCircle(140, 140, 50, TFT_COLOR_ILI9341_GREEN);


	  for (int i=0; i < dl_n; i++){
		  TFTDisplay_ILI9341_DrawChar (nx, ny, *ptr);
		  nx= nx +10;
		  ptr++;

	  }


	  TFTDisplay_ILI9341_DrawLine(0, 0, 240, 240, TFT_COLOR_ILI9341_RED);
	  TFTDisplay_ILI9341_DrawPixel(140, 140, TFT_COLOR_ILI9341_WHITE);

*/




  /* Infinite loop */
  for(;;)
  {
      prepreScale++;

              if (prepreScale == 1)
              {
                  preScale++;
                  prepreScale = 0;

                  if (preScale % 1000 == 0) //wywolana co 1[sek] = 1000[ms]
                  {

                      unsigned char napisCzas[] = {"Czas[s] = _____"};

                      napisCzas[11] = (Tim / 1000 + '0');
                      napisCzas[12] = (Tim / 100 - 10 * (Tim / 1000) + '0');
                      napisCzas[13] = Tim / 10 - 10 * (Tim / 100) + '0';
                      napisCzas[14] = Tim % 10 + '0';

                    //  PrintText(textEkran, Text, 15, 25, 0);

                      ptr=napisCzas;
                   //   TFTDisplay_ILI9341_DrawChar (10, 10, *ptr);
                      Tim++;

                      if(trybPracy == 1)
                          Wykresy();
                  }

                  /*
                  1. (wywolana raz na poczatku) wywołuj funkcję analiza i regulacja
                      jesli preScale==1 -> koniecRysowania=FALSE + wywołaj regulację + zapisz czas konca regulacji (preScaleZapisany)
                  2. co 20[ms] wywołuj funcję rysuj
                      jesli (preScale-preScaleZapisany)%20==0 && !koniecRysowania -> wywolaj rysowanie
                      jesli koniec rysowania zmien zmienna -> koniecRysowania=TRUE
                  3. (wywolana raz na koniec) delay na 1000[ms] - przerwa miedzy lotami
                  4. (wywolana raz na koniec) reset zmiennej preScale
                  */

                  if (preScale == 1)
                  {
                      koniecAnalizy = false;
                      koniecRysowania = false;
                      preScaleZapisany = preScale;
                    //  TFTDisplay_ILI9341_FillRect(0, 0, 120, 60, TFT_COLOR_ILI9341_YELLOW);
                      if(GPIOA -> IDR & 0x0001) {

                          setCelReg(getCelReg()+10);
                          if(getCelReg()>=229) setCelReg(15);
                        }


                      TFTDisplay_ILI9341_FillRect(0, 0, 216, 319, TFT_COLOR_ILI9341_BLUE);
                      TFTDisplay_ILI9341_FillRect(216, 0, 239, 319, TFT_COLOR_ILI9341_GREEN);
                   //   TFTDisplay_ILI9341_FillRect(1, 1, szeEkranu-1, offsetPodloza, TFT_COLOR_ILI9341_GREEN);
                      osDelay(4);
                      for (unsigned int w = 0; w < wysEkranu; w++)
                                {
                                    for (unsigned int s = 0; s < szeEkranu; s++)
                                    {
                                        //podloze
                                      //  if (w <= offsetPodloza)
                                          //  if(trybPracy == 0)
                                                //SetPixel(ekran, s, (wysEkranu - w - 1));
                                          //      TFTDisplay_ILI9341_DrawPixel((wysEkranu - w - 1), s, TFT_COLOR_ILI9341_GREEN);
                                          //      TFTDisplay_ILI9341_FillRect(0, 0, szeEkranu, offsetPodloza, TFT_COLOR_ILI9341_RED);

                                        //punkt docelowy
                                        if (abs((int)(w - offsetPodloza)) <= 5 && abs((int)(s - getCelReg())) <= 4)
                                            if(trybPracy == 0)
                                                //SetPixel(ekran, s, (wysEkranu - w - 1 -3));
                                                TFTDisplay_ILI9341_DrawPixel((wysEkranu - w - 1 -3), s, TFT_COLOR_ILI9341_BLACK);
                                        //armata
                                        /*
                                        if(s>=5 && s<=(5+dlugoscA) && w >=(5+offsetPodloza) && w<=(5+offsetPodloza+szerokoscA))
                                        {
                                            int xA=s*cos(stopnieNaRad(objReg.getKat()));
                                            cout<<xA;
                                            int yA=w*sin(stopnieNaRad(objReg.getKat()));
                                            cout<<yA;
                                            SetPixel(ekran, xA, (wysEkranu-yA));
                                            //SetPixel(ekran, s, (wysEkranu-w));
                                        }
                                        */
                                    }
                                }




                  }

                  if (!koniecRysowania && (preScale - preScaleZapisany) % czasAnimacji == 0)
                  {
                      RysowanieTrajektorii();
                  }

                  if (koniecRysowania && preScale == (preScaleZapisany + 2000))
                  {
                      AnalizaIRegulacja();
                      preScale = 0;
                     // if(trybPracy == 0)       TFTDisplay_ILI9341_FillScreen(TFT_COLOR_ILI9341_WHITE);
                      //DodajDaneZadana((int)getCelReg());
                  }
      /*
                  unsigned char Key = KEYBOARD.GetKey();

                  if (Key == 6) objReg.setCelReg(objReg.getCelReg() + 0.1); //przycisk 6
                  if (Key == 8) objReg.setCelReg(objReg.getCelReg() - 0.1); //przycisk *

                  if (Key == 4)  trybPracy=WYKRESY; //przycisk 5
                  if (Key == 2) trybPracy=RYSOWANIE; //przycisk C
      */
      /*
                  unsigned char Text1[] = "Wartosc zadana______";

                      for (int j = 0; j < rozmiar; j++)
                      {
                          Komentarze[j] = Text1[j];
                      }

                      int celR = (int)objReg.getCelReg();

                      Komentarze[17] = (celR / 100 - 10 * (celR / 1000) + '0');
                      Komentarze[18] = celR / 10 - 10 * (celR / 100) + '0';
                      Komentarze[19] = celR % 10 + '0';

                  PrintText(textEkran, Komentarze, 20, 0, 5);
      */
              }


      //osDelay(4);
/*
	  	TFTDisplay_ILI9341_FillRect(x, y, x+30, y+30, TFT_COLOR_ILI9341_RED);
	  	osDelay(4);
	  	move_square_C();
	  	TFTDisplay_ILI9341_FillRect(x_old, y_old, x_old+30, y_old+30, TFT_COLOR_ILI9341_BLUE);
	  	x_old = x;
	  	y_old = y;
*/
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskLED1 */
/**
* @brief Function implementing the LED1_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLED1 */
void StartTaskLED1(void *argument)
{
  /* USER CODE BEGIN StartTaskLED1 */
  /* Infinite loop */
  for(;;)
  {

	  osSemaphoreAcquire(BinarySem01Handle, osWaitForever);  //synchronizacja tasków - zwolnienie w TaskLED2

	  LL_GPIO_TogglePin(GPIOG, LL_GPIO_PIN_13);


	  osDelay(100);
  }
  /* USER CODE END StartTaskLED1 */
}

/* USER CODE BEGIN Header_StartTaskLED2 */
/**
* @brief Function implementing the LED2_tack thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLED2 */
void StartTaskLED2(void *argument)
{
  /* USER CODE BEGIN StartTaskLED2 */
  /* Infinite loop */
  for(;;)
  {


	  LL_GPIO_TogglePin(GPIOG, LL_GPIO_PIN_14);

	  osDelay(1000);
	  osSemaphoreRelease(BinarySem01Handle);	//zwolnienie semafora i odblokowanie TaskLED1

	  ;
  }
  /* USER CODE END StartTaskLED2 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

