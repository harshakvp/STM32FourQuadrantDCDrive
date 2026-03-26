/* USER CODE BEGIN Header */
/**
 * @file    main.c
 * @brief   Main application for four-quadrant DC motor control
 *
 * This firmware implements:
 *  - Forward and reverse directional control
 *  - Speed control using a potentiometer
 *  - Toggle-based push button operation
 *  - Software dead-time and safety delay for switching protection
 *
 * Target MCU : STM32F446RE
 * Framework  : STM32 HAL
 */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
/* Motor operating states */
typedef enum {
    STOP = 0,
    FORWARD,
    REVERSE
} Direction;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
/* Timing constants for safe motor operation */
#define DEADTIME_US      100
#define SAFETY_DELAY     500
#define DEBOUNCE_DELAY   200
/* USER CODE END PD */

/* Peripheral handles */
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* Runtime state variables */
Direction currentDir = STOP;
bool isRunning = false;

/* Control variables */
uint32_t pwmVal = 0;
uint32_t lastDebounce = 0;
uint32_t filteredADC = 0;
/* USER CODE END PV */

/* Function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
/* Application-level function prototypes */
void delay_us(uint16_t us);
uint32_t readADC(void);
uint32_t readADC_Averaged(void);
void runForward(uint32_t pwm);
void runReverse(uint32_t pwm);
void stopMotor(void);
void applyBrake(void);
void loop_logic(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/**
 * @brief  Generates blocking delay in microseconds using DWT cycle counter
 * @param  us Delay duration in microseconds
 */
void delay_us(uint16_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000U);
    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}

/**
 * @brief  Reads one fresh ADC sample from the potentiometer
 * @retval ADC value in the range 0 to 4095
 */
uint32_t readADC(void)
{
    HAL_ADC_Start(&hadc1);                         // Start fresh conversion
    HAL_ADC_PollForConversion(&hadc1, 10);
    return HAL_ADC_GetValue(&hadc1);              // 12-bit ADC result
}

/**
 * @brief  Averages multiple ADC samples for smoother speed control
 * @retval Averaged ADC value
 */
uint32_t readADC_Averaged(void)
{
    uint32_t sum = 0;

    for (int i = 0; i < 8; i++)
    {
        sum += readADC();
    }

    return (sum / 8);
}

/**
 * @brief  Drives the motor in forward direction
 * @param  pwm PWM duty value (0 to 4095)
 *
 * Switching state:
 *  - INAHI = PWM
 *  - INBHI = LOW
 *  - INALO = LOW
 *  - INBLO = HIGH
 */
void runForward(uint32_t pwm)
{
    if (pwm > 4095) pwm = 4095;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);       // INBHI LOW
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // INALO LOW

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);     // INAHI PWM
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);    // INBLO HIGH
}

/**
 * @brief  Drives the motor in reverse direction
 * @param  pwm PWM duty value (0 to 4095)
 *
 * Switching state:
 *  - INAHI = LOW
 *  - INBHI = PWM
 *  - INALO = HIGH
 *  - INBLO = LOW
 */
void runReverse(uint32_t pwm)
{
    if (pwm > 4095) pwm = 4095;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);       // INAHI LOW
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  // INBLO LOW

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);     // INBHI PWM
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    // INALO HIGH
}

/**
 * @brief  Turns OFF all bridge switches safely
 *
 * Used during:
 *  - Stop condition
 *  - Direction transition
 *  - Brake condition
 */
void stopMotor(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    delay_us(DEADTIME_US);
}

/**
 * @brief  Applies brake action
 *
 * Current implementation performs coast stop
 * by turning OFF all switches and holding for 500 ms.
 */
void applyBrake(void)
{
    stopMotor();
    HAL_Delay(500);
}

/**
 * @brief  Main application control logic
 *
 * Handles:
 *  - Potentiometer speed acquisition
 *  - Button debounce and toggle logic
 *  - Direction selection
 *  - Brake action
 *  - Motor output update
 */
void loop_logic(void)
{
    /* Read filtered speed reference */
    filteredADC = readADC_Averaged();
    pwmVal = filteredADC;

    /* Forward button toggle */
    if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) &&
        (HAL_GetTick() - lastDebounce > DEBOUNCE_DELAY))
    {
        lastDebounce = HAL_GetTick();

        if (currentDir != FORWARD)
        {
            stopMotor();
            HAL_Delay(SAFETY_DELAY);
            currentDir = FORWARD;
            isRunning = true;
        }
        else
        {
            currentDir = STOP;
            isRunning = false;
            stopMotor();
        }
    }

    /* Reverse button toggle */
    if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) &&
        (HAL_GetTick() - lastDebounce > DEBOUNCE_DELAY))
    {
        lastDebounce = HAL_GetTick();

        if (currentDir != REVERSE)
        {
            stopMotor();
            HAL_Delay(SAFETY_DELAY);
            currentDir = REVERSE;
            isRunning = true;
        }
        else
        {
            currentDir = STOP;
            isRunning = false;
            stopMotor();
        }
    }

    /* Brake button action */
    if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) &&
        (HAL_GetTick() - lastDebounce > DEBOUNCE_DELAY))
    {
        lastDebounce = HAL_GetTick();

        applyBrake();
        currentDir = STOP;
        isRunning = false;
    }

    /* Execute motor control state */
    if (isRunning)
    {
        if (currentDir == FORWARD)
            runForward(pwmVal);
        else if (currentDir == REVERSE)
            runReverse(pwmVal);
    }
    else
    {
        stopMotor();
    }
}

/* USER CODE END 0 */

/**
 * @brief  Main program entry point
 * @retval int
 */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();

    /* USER CODE BEGIN 2 */

    /* Start PWM generation on both high-side channels */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    /* Start ADC peripheral */
    HAL_ADC_Start(&hadc1);

    /* Enable DWT cycle counter for microsecond timing */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Ensure safe bridge state at startup */
    stopMotor();

    /* USER CODE END 2 */

    while (1)
    {
        loop_logic();
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  Initializes ADC1 for potentiometer input sampling
 * @retval None
 */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;   // Manual conversion mode
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;   // Improved stability for potentiometer

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  Initializes TIM1 for PWM generation
 * @retval None
 */
static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 20;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 4095;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief  Initializes GPIO pins for buttons, outputs, and status signals
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Safe default output state */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /* User push-buttons */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART2 pins generated by CubeMX */
    GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* On-board LED */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    /* Low-side H-bridge control outputs */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief  Handles unrecoverable system errors
 *
 * All motor drive outputs are forced OFF before entering fail-safe loop.
 */
void Error_Handler(void)
{
    __disable_irq();

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    while (1)
    {
    }
}
