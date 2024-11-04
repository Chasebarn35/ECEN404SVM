/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32g4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
#include "algos.h"
#include "arm_math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;




/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1)
	{
	}
	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line2 interrupt.
  *///EXTI2 has Short
void EXTI2_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(Short_Pin)!=RESET)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(Short_Pin);
		HAL_GPIO_WritePin(GPIOC,Start_Pin,GPIO_PIN_RESET);//TURNS OFF START
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
	}
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}


/**
 * @brief This function handles EXTI line[9:5] interrupts.
 *///EXTI9_5 has V_CA
void EXTI9_5_IRQHandler(void)
{

	if(__HAL_GPIO_EXTI_GET_IT(V_C_Pin)!=RESET){ 
		ThetaCA = 0;
		ThetaC = _PI330;
		ThetaA = _PI210;
		ThetaB = _PI_90;
		__HAL_GPIO_EXTI_CLEAR_IT(V_C_Pin);
	}
}

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 *///EXTI_15_10 has V_AB, V_BC
void EXTI15_10_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(V_A_Pin)!=RESET){
		ThetaAB = 0;
		ThetaA = _PI330;
		ThetaB = _PI210;
		ThetaC = _PI_90;

		__HAL_GPIO_EXTI_CLEAR_IT(V_A_Pin);
	}

	if(__HAL_GPIO_EXTI_GET_IT(V_B_Pin)!=RESET){ 
		ThetaBC = 0;
		ThetaB = _PI330;
		ThetaC = _PI210;
		ThetaA = _PI_90;
		__HAL_GPIO_EXTI_CLEAR_IT(V_B_Pin);
	}

	//INITIALIZE STARTUP AND WRITE 1 TO START PIN
	if(__HAL_GPIO_EXTI_GET_IT(B1_Pin)!=RESET){
		if(!hasStarted){
			HAL_GPIO_WritePin(GPIOA,LED_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, Start_Pin,GPIO_PIN_SET);//TURN ON START
			hasStarted = 1;
			__HAL_GPIO_EXTI_CLEAR_IT(B1_Pin);
		}
	}

	/* USER CODE END EXTI15_10_IRQn 0 */
	/* USER CODE BEGIN EXTI15_10_IRQn 1 */

	/* USER CODE END EXTI15_10_IRQn 1 */
}

/**
 * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
 */
//10KHz interrupt
void TIM6_DAC_IRQHandler(void)
{
	//HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n", 2, 0xFFFF);//LOL LMAO
	if(TIM_GET_ITSTATUS(&htim6,TIM_IT_UPDATE)!=RESET){
		if(_DEBUG_FLAG){
		currVec+=21;
		if(currVec>63)
			currVec=0;
		}	
		else{
		currVec = Mat[0] | Mat[3] << 1 | Mat[6]*3 | Mat[1] << 2 | Mat[4]<<3 | Mat[7]*3 << 2 | Mat[2] << 4 | Mat[5] << 5 | Mat[8]*3 << 4;
		}
		PhaseChange(currVec);
		TIM_GET_CLEAR_IT(&htim6,TIM_IT_UPDATE);
	}
}

/**
 * @brief This function handles TIM7 global interrupt.
 */
//50KHz Interrupt
void TIM7_IRQHandler(void)
{

	if(TIM_GET_ITSTATUS(&htim7,TIM_IT_UPDATE)!=RESET){

		ThetaAB+=(60*2*M_PI)/50000;//Reset via V_A interrupt
		ThetaBC+=(60*2*M_PI)/50000;//Reset via V_B interrupt
		ThetaCA+=(60*2*M_PI)/50000;//Reset via V_C interrupt
		ThetaA+=(60*2*M_PI)/50000;//set via 0Crossing interrupts
		ThetaB+=(60*2*M_PI)/50000;//set via 0Crossing interrupts
		ThetaC+=(60*2*M_PI)/50000;//set via 0Crossing interrupts
		ThetaOut+=(DFreq*2*M_PI)/50000;//DFreq Increment every 50Khz

		//Reset Output and Line to Neutral Thetas
		if(ThetaOut >= (2*M_PI))
			ThetaOut = 0;
		if(ThetaA >= (2*M_PI))
                        ThetaA = 0;
                if(ThetaB >= (2*M_PI))
                        ThetaB = 0;
                if(ThetaC >= (2*M_PI))
                        ThetaC = 0;
		

		//2.5kHz Triangle Carrier Wave
		switch(triangle){
		case 1:
		triangleWave += 2500/50000;
		if(triangleWave >=1) 
			triangle = 0;
		break;
		case 0:
		triangleWave -= 2500/50000;
                if(triangleWave <=0)
                        triangle = 1;
		break;
		}

		//Set the Values for the 3 voltages
		sin_ab = arm_sin_f32(ThetaAB);
		sin_bc = arm_sin_f32(ThetaBC);
		sin_ca = arm_sin_f32(ThetaCA);

		//virtual input of A
		if(ThetaA >= _PIdiv6 && ThetaA < _5PIdiv6){
			Mat[0] = 1;
			Mat[3] = 0;
			Mat[6] = 0;
			V_IN[0] = sin_ab;
			V_IN[1] = sin_bc;
			V_IN[2] = sin_ca;
			virt_a = &VMat[0];
			virt_b = &VMat[3];	
			virt_c = &VMat[6];
		}//Virtual input of B as max
		else if(ThetaB >= _PIdiv6 && ThetaB < _5PIdiv6){
			Mat[0] = 0;
			Mat[3] = 1;
			Mat[6] = 0;
			V_IN[0] = sin_bc;
			V_IN[1] = sin_ca;
			V_IN[2] = sin_ab;
			virt_a = &VMat[3];
			virt_b = &VMat[6];
			virt_c = &VMat[0];
		}//Virtual input of C as max
		else{
			Mat[0] = 0;
			Mat[3] = 0;
			Mat[6] = 1;
			V_IN[0] = sin_ca;//TODO VERIFY THIS STUFF
			V_IN[1] = sin_ab;
			V_IN[2] = sin_bc;
                        virt_a = &VMat[6];
			virt_b = &VMat[0];
			virt_c = &VMat[3];
		}


		//Virtual Line to Line input	
		V_AB = _U_DC * V_IN[0];
		V_BC = _U_DC * V_IN[1];
		V_CA = _U_DC * V_IN[2];

		DENOM = V_AB*V_AB + V_CA*V_CA + V_BC*V_BC;
		//Virtualized AB Output
		cosinevalue = arm_cos_f32(ThetaOut + _PIdiv3);
		v_ab = -_U_OUT * _SQRT3 * cosinevalue;//output V_AB
		//Virtualized AC Output
		cosinevalue = arm_cos_f32(ThetaOut + 2*_PIdiv3);
		v_ac = -_U_OUT * _SQRT3 * cosinevalue;//output V_AC //TODO DETERMINE _U_OUT
		virt_a[0] = 1;
		virt_b[0] = 0;
		virt_c[0] = 0;
		virt_b[1] = ((V_AB-V_BC)*v_ab)/DENOM;//TODO DETERMINE SPEED OF CALCULATIONS
		virt_b[2] = ((V_AB-V_BC)*v_ac)/DENOM;
		virt_c[1] = ((V_BC-V_CA)*v_ab)/DENOM;
		virt_c[2] = ((V_BC-V_CA)*v_ac)/DENOM;
		virt_a[1] = 1 - virt_b[1] - virt_c[1];
		virt_a[2] = 1 - virt_b[2] - virt_c[2];

		Mat[1] = VMat[1] > triangleWave ? 1 : 0;
		Mat[2] = VMat[2] > triangleWave ? 1 : 0;
		Mat[7] = VMat[7] > 1 - triangleWave ? 1 : 0; 
		Mat[8] = VMat[8] > 1 - triangleWave ? 1 : 0; 
		Mat[4] = (!Mat[1] && !Mat[7])?1:0;
		Mat[5] = (!Mat[2] && !Mat[8])?1:0;

		TIM_GET_CLEAR_IT(&htim7,TIM_IT_UPDATE);
	}
}



/**
 * @brief This function handles LPUART1 global interrupt.
 */

uint8_t isFreqMode = 0;
uint8_t bufferIndex = 0; 
void LPUART1_IRQHandler(void)
{
	/* USER CODE BEGIN LPUART1_IRQn 0 */

	uint8_t rxData;
	uint8_t txString[100];//TODO VERIFIY THATS THE MAX SIZE
	uint8_t txStrSize;
	char   freqStr[5];
	uint8_t BUFSIZE = 6;
	uint8_t rxBuffer[BUFSIZE];

	if(__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_RXNE)!=RESET){
                rxData = hlpuart1.Instance->RDR;
		if(!isFreqMode){	
		switch(rxData){ 
			case 'h':
				snprintf((char*)txString,88,"Usage:\r\n h\t help \r\n d\t output frequency\r\n m\t matrix\r\n t\t thetaC\r\n f\t change frequency\r\n");
				txStrSize = 88;//TODO VERIFY
			break;
			case 'd':
				snprintf(freqStr,5,"%.2f",DFreq);	
				snprintf((char*)txString,27, "Current frequency:\t %s\r\n", freqStr);
				txStrSize = 26; 
			break;
			case 'f':
				snprintf((char*)txString,24,"Frequency set to: ");
				txStrSize = 18;
				snprintf((char*)rxBuffer, BUFSIZE, "000000");
				isFreqMode = 1;//cheap bool flag
			break;
			case 'c':
				snprintf(freqStr,10,"%.4f",ThetaOut);
				snprintf((char*)txString,27, "Current ThetaOut:\t %s\r\n", freqStr);
				txStrSize = 28; 
			break;
			case 'm':
				snprintf((char*)txString,35,"[%d %d %d ] \r\n[%d %d %d ] \r\n[%d %d %d ] \r\n\n",Mat[0], Mat[1], Mat[2],Mat[3], Mat[4], Mat[5], Mat[6],Mat[7], Mat[8]);
				txStrSize = 35;	
			break;
			default:
				snprintf((char*)txString,18,"Invalid command\n\r");
				txStrSize = 18;
			break;
		}
		HAL_UART_Transmit(&hlpuart1, txString, txStrSize, 0xFFFF);
		}
		else{//TODO READING MULTIPLE RXDATAS UNTIL END OF INPUT
		switch(rxData){
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
                        case '6':
                        case '7':
                        case '8':
                        case '9':
                        case '0':
                        case '.':
			if(bufferIndex < BUFSIZE - 1){
				rxBuffer[bufferIndex++] = rxData;
				rxBuffer[bufferIndex] = '\0';
				HAL_UART_Transmit(&hlpuart1, &rxData, 1, 0xFFFF);	
			}
			break;
                        case '\r':
                        case '\n':
				DFreq = (float32_t)atof((char*)rxBuffer); //TODO VERIFY THIS WORKS
				HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\n\r", 2, 0xFFFF);	
                                isFreqMode = 0;
				bufferIndex = 0;
                        break;
                        default:
				//do nothing 
                        break;
		}



		}
		__HAL_UART_CLEAR_FLAG(&hlpuart1, UART_FLAG_RXNE);
        }
			

	/* USER CODE END LPUART1_IRQn 0 */
	HAL_UART_IRQHandler(&hlpuart1);
	/* USER CODE BEGIN LPUART1_IRQn 1 */

	/* USER CODE END LPUART1_IRQn 1 */
}


int __io_putchar(int ch)
{
    /* Support printf over UART */
    (void) HAL_UART_Transmit(&hlpuart1, (uint8_t *) &ch, 1, 0xFFFF);
    return ch;
}


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
