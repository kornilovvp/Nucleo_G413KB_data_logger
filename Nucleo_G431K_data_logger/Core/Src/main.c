/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include <stdio.h>
#include <string.h>


#include "NanoEdgeAI.h"
#include "knowledge.h"




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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */



//#define UART_SPEED 115200
#define UART_SPEED 921600  // 104 uSEc
//#define UART_SPEED 1843200 // 52  uSec 


//#define ROW_LEN  (1536)

//uint32_t row_count = 0;
//uint32_t raw_adc1_ch1_val = 0;
//uint32_t raw_adc1_ch2_val = 0;
//uint32_t raw_adc1_ch3_val = 0;
//float rawf_adc1_ch1_val = 0;
//uint8_t msg_bug[32];
//int msg_len = 0;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */






/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */







//uint32_t dbg_daq_reads_per_1ec = 0;
//uint32_t cnt_daq_reads_int = 0;



typedef struct 
{
    uint32_t raw_updated;
    uint32_t raw_vref;
    uint32_t raw_vbat;
    uint32_t raw_vtemp;
    uint32_t raw_vin;
    
    uint32_t v_onebit_mv;
    uint32_t v_power;
    uint32_t v_ref;
    //uint32_t v_in_mv;

}_adc;

_adc adc_data;





typedef struct 
{
    uint32_t dbg_adc1_int_per_1ec;
    uint32_t dbg_adc1_int;

    uint32_t dbg_adc2_int_per_1ec;
    uint32_t dbg_adc2_int;
    
    
}_dbg;

_dbg debug;





#define INERNAL_VREF_mV  1213  // 1.212V


// Buffer for ADC1
uint16_t adc1_raw_data[6];


//float Vref = 3.3f;
//float Vzero = Vref/2.0f;
//float Vcode = 0.0f;
float Vresult = 0.0f;
  



/*
#define ADC_DATA_EMPTY  0
#define ADC_DATA_FULL 10
uint8_t adc_data_buf1 = ADC_DATA_EMPTY;
uint8_t adc_data_buf2 = ADC_DATA_EMPTY;

#define DMA_LOCK_STOP 0
#define DMA_LOCK_BUF1 10
#define DMA_LOCK_BUF2 20

uint8_t dma_buffer_lock = DMA_LOCK_STOP;
*/

// Buffer for ADC2
uint16_t adc2_raw_data_buf1[DATA_INPUT_USER*AXIS_NUMBER];
//uint16_t adc2_raw_data_buf2[DATA_INPUT_USER*AXIS_NUMBER];


uint8_t  adc_data_ready = 0;
uint16_t tmp_buff[DATA_INPUT_USER];





uint8_t LedState = 0;;




void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        debug.dbg_adc1_int++;
        
        adc_data.raw_updated = 1;
        adc_data.raw_vtemp = adc1_raw_data[0];
        adc_data.raw_vref  = adc1_raw_data[1];
        adc_data.raw_vbat  = adc1_raw_data[2];
        
        //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
    
    
   if (hadc == &hadc2)
    {
        HAL_ADC_Stop_DMA(&hadc2);
      
        
        // DEBUG
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        
        
        debug.dbg_adc2_int++;
    
        // copy data to temp buf
        for ( uint32_t i = 0; i < DATA_INPUT_USER; i++ )
        {
            tmp_buff[i] = adc2_raw_data_buf1[i];
        }
        
        adc_data_ready = 1;
        
        // DEBUG
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        
        
        HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_raw_data_buf1, DATA_INPUT_USER);
    }
    
    
    
}





void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    __NOP();
    __NOP();
    __NOP();
    __NOP();  
  
  
  /*

  
 */ 

  
  
  /*
   if (hadc == &hadc2)
    {
        HAL_ADC_Stop_DMA(&hadc2);
      
        debug.dbg_adc2_int++;

        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
   */     
        
        /*
        if (dma_buffer_lock == DMA_LOCK_BUF1)
        {
        
            // notify buff1 ready
            
            adc_data_buf1 = ADC_DATA_FULL;
            
            
            // switch to buff 2
            
            if (adc_data_buf2 == ADC_DATA_FULL)
            {
                // user does not clear current buffer, something wrong
                while(1)
                {
                    __NOP();
                    __NOP();
                    __NOP();
                    __NOP();
                }
            }
            
            // Start adc on buffer 2
            
            dma_buffer_lock = DMA_LOCK_BUF2;
            
            HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_raw_data_buf2, DATA_INPUT_USER);
          
        }
        else if (dma_buffer_lock == DMA_LOCK_BUF2)
        {
            // notify buff2 ready
            
            adc_data_buf2 = ADC_DATA_FULL;
            
            
            // switch to buff 1
            
            if (adc_data_buf1 == ADC_DATA_FULL)
            {
                // user does not clear current buffer, something wrong
                while(1)
                {
                    __NOP();
                    __NOP();
                    __NOP();
                    __NOP();
                }
            }   
            
            // Start adc on buffer 1
            
            dma_buffer_lock = DMA_LOCK_BUF1;
            
            HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_raw_data_buf1, DATA_INPUT_USER);
        }
        */
        
        
        
    //}

}










void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 1 sec
    if (htim == &htim7)
    {
      
        // debug adc 1
        debug.dbg_adc1_int_per_1ec = debug.dbg_adc1_int;
        
        debug.dbg_adc1_int = 0;
        
        
        // debug adc 2
        debug.dbg_adc2_int_per_1ec = debug.dbg_adc2_int;
        
        debug.dbg_adc2_int = 0;  
        
        
        //debug 
        //dbg_daq_reads_per_1ec = cnt_daq_reads_int;
        
        //cnt_daq_reads_int = 0;
        


        //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        
        LedState = !LedState;
    }

}



////////////////////////////////////////////////////////////////////////////////


// AI
#define LEARNING_ITERATIONS 20

float input_user_buffer[DATA_INPUT_USER * AXIS_NUMBER]; // 1536 * 1 = 1536
// Buffer of input values 




void fill_buffer(uint16_t input_buffer[])
{
    adc_data.v_onebit_mv = (INERNAL_VREF_mV * 1000) / adc_data.raw_vref;
    
    //adc_data.v_in_mv    = (adc_data.v_onebit_mv * adc_data.raw_vin ) / 1000;
  
    for (uint32_t i = 0; i < DATA_INPUT_USER; i++)
    {
        input_user_buffer[i] = ( adc_data.v_onebit_mv * (float)input_buffer[i] ) / 1000 / 1000;   // uV -> mV -> V
    }
}





enum neai_state error_code;
uint8_t similarity = 0;  

uint32_t adc_buff_counter = 0;


enum neai_state neai_learn_state;




uint32_t button_selected = 0;

uint32_t fl_ai_new_learn = 0;
uint32_t ai_treining_rounds = 0;



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  {
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  
  
  
  // ai


  error_code = neai_anomalydetection_init();

  if (error_code != NEAI_OK) 
  {
      /* This happens if the library works into a not supported board. */
      __NOP();
      __NOP();
      __NOP();
      __NOP();
  }

  //neai_learn_state = neai_anomalydetection_knowledge(knowledge);   
      
  if (error_code != NEAI_OK) 
  {
      __NOP();
      __NOP();
      __NOP();
      __NOP();
  }
    
    
    
    
  
  // 1 sec
  HAL_TIM_Base_Start_IT(&htim7); 
  
  
  // 1 msec
  HAL_TIM_Base_Start(&htim6);  
  
  
  }
  
  // Start ADC1
  
  
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  HAL_Delay(100);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_raw_data, 3);
  
  while( adc_data.raw_updated == 0 ){  }  

  

  
  
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  
  HAL_Delay(100);
  
  
  /*
  adc_data_buf1 = ADC_DATA_EMPTY;
  adc_data_buf2 = ADC_DATA_EMPTY;
  
  dma_buffer_lock = DMA_LOCK_BUF1;
  */
  
  
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_raw_data_buf1, DATA_INPUT_USER);
  
  // Disable half receive adc callback
  __HAL_DMA_DISABLE_IT(hadc2.DMA_Handle , DMA_IT_HT);
  
  //HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_raw_data_buf1, 100);
  
  

  

    
    
    /* Learning process ----------------------------------------------------------*/
    
    /*

    for (uint16_t iteration = 0 ; iteration < LEARNING_ITERATIONS ; iteration++) 
    {
        fill_buffer(input_user_buffer);
        
        neai_learn_stateneai_anomalydetection_learn(input_user_buffer);
    }

    */
  
    /*

    // while(1)

    fill_buffer(input_user_buffer);
     
    neai_anomalydetection_detect(input_user_buffer, &similarity);

     */
        

        
  
  while(1)
  {
  
    // weating data for work
    
    
    // button detecting
    if ( HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET)
    {
        fl_ai_new_learn = 1;
    }
    else
    {
        if ( fl_ai_new_learn == 1)
        {
            fl_ai_new_learn = 2;
        }
        

    }
    
    
    
    static uint32_t learn_mode = 25;
    
    
    if ( adc_data_ready )
    {
        adc_data_ready = 0;
    
    
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        
        fill_buffer(tmp_buff);
        
        
        
        
        
        if ( fl_ai_new_learn == 2 )
        {
            fl_ai_new_learn = 0;
            
            learn_mode = 25;
        }
        
        
        
        
        if (learn_mode)
        {
            if ( learn_mode == 25)
            {
                neai_anomalydetection_init();
            }
          
            learn_mode --;
            
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            
            error_code = neai_anomalydetection_learn(input_user_buffer);
            
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
          
        }
        else
        {
        
            error_code = neai_anomalydetection_detect(input_user_buffer, &similarity);
            
            
            if (similarity < 95)
            {
                HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            }    
        
        }
        
        
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    
    }
    
    
    
    
    // Test buffer 1
    /*
    if ( ( adc_data_buf1 == ADC_DATA_FULL ) || ( adc_data_buf2 == ADC_DATA_FULL ) )
    {
      
        if ( adc_data_buf1 == ADC_DATA_FULL )
        {
            // move data to ai buffer
            fill_buffer(adc2_raw_data_buf1);
            
            // Mark buffr as free
            adc_data_buf1 = ADC_DATA_EMPTY;
        }
      
        
        // Test buffer 2
        if (adc_data_buf2 == ADC_DATA_FULL)
        {
            // move data to ai buffer
            fill_buffer(adc2_raw_data_buf2);
          
            // MArk buffr as free
            adc_data_buf2 = ADC_DATA_EMPTY;
        }
        
        
        
        
        neai_learn_state = neai_anomalydetection_detect(input_user_buffer, &similarity);
        
        if (similarity < 95)
        {
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        }
        

    }
    */

    
  }
     
     
  
  while (1)
  {
    
    
      // Port marker HI
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      
      __HAL_TIM_SET_COUNTER(&htim6, 0);
      
      
      
      
      
      //adc_data.v_onebit_mv = (INERNAL_VREF_mV * 1000) / adc_data.raw_vref;
      
      //adc_data.v_in_mv    = (adc_data.v_onebit_mv * adc_data.raw_vin ) / 1000;
      
      //Vresult =  (double)adc_data.v_in_mv / 1000;
        
        
        
      
      //rawf_adc1_ch1_val = Vresult;  //HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);

      
      
      
      
      
      // button detecting
      if ( HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET)
      {
          fl_ai_new_learn = 1;
      }
      else
      {
          fl_ai_new_learn = 0;
          
          ai_treining_rounds = 0;
      }
    
    
    
        
    
    
    

      // ai detection
      
      
      //input_user_buffer[adc_buff_counter] = rawf_adc1_ch1_val;
      
      adc_buff_counter ++;
      
      /*
      if (adc_buff_counter >= DATA_INPUT_USER)
      {
          adc_buff_counter = 0;
          
          

          
          
          
          HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
          
          
          
          
          // new learn function
          if ( fl_ai_new_learn )
          {

          
              
              //for (uint16_t iteration = 0 ; iteration < LEARNING_ITERATIONS*5 ; iteration++) 
              //{
                  //fill_buffer(input_user_buffer);
                  
              neai_learn_state = neai_anomalydetection_learn(input_user_buffer);
              
              //}
              
              ai_treining_rounds++;
              
              __NOP();
              __NOP();
              __NOP();
              __NOP();
          }
          else
          {
          
          
              // ai detection
              neai_learn_state = neai_anomalydetection_detect(input_user_buffer, &similarity);
              
              if (similarity < 95)
              {
                  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
              }
              else
              {
                  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
              }
      
          }
          
          
          
          HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
      }
      
      */
      

      /*
      
      
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);  
      
      //cnt_daq_reads_int++;
      
      while( __HAL_TIM_GET_COUNTER(&htim6) < 33 )  // 330 usec delay
      {    
      }

    */
    
    
    
    
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_8;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_VBAT;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetSign = ADC_OFFSET_SIGN_POSITIVE;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_8;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1699;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 16999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  //#warning please sen instade 115200 the defs UART_SPEED
  
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_G_Pin|LED_R_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_G_Pin LED_R_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin|LED_R_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
