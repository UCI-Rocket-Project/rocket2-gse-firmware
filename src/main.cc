#include "main.h"

#include <cmath>

#include "crc.h"
#include "tc_max31855_spi.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#pragma pack(push, 1)
struct AdcData {
    uint32_t s4;
    uint32_t s5;
    uint32_t s6;
    uint32_t s7;
    uint32_t s8;
    uint32_t pt0;
    uint32_t pt1;
    uint32_t pt2;
    uint32_t pwr0;
    uint32_t pwr1;
    uint32_t s0;
    uint32_t s1;
    uint32_t s2;
    uint32_t s3;
    uint32_t pt3;
    uint32_t pt4;
};

struct GseCommand {
    bool igniter0Fire;
    bool igniter1Fire;
    bool alarm;
    bool solenoidStateGn2Fill;
    bool solenoidStateGn2Vent;
    bool solenoidStateGn2Disconnect;
    bool solenoidStateMvasFill;
    bool solenoidStateMvasVent;
    bool solenoidStateMvasOpen;
    bool solenoidStateMvasClose;
    bool solenoidStateLoxVent;
    bool solenoidStateLngVent;
    uint32_t crc;
};

struct GseData {
    uint32_t timestamp;
    bool igniterArmed;
    bool igniter0Continuity;
    bool igniter1Continuity;
    bool igniterInternalState0;
    bool igniterInternalState1;
    bool alarmInternalState;
    bool solenoidInternalStateGn2Fill;
    bool solenoidInternalStateGn2Vent;
    bool solenoidInternalStateGn2Disconnect;
    bool solenoidInternalStateMvasFill;
    bool solenoidInternalStateMvasVent;
    bool solenoidInternalStateMvasOpen;
    bool solenoidInternalStateMvasClose;
    bool solenoidInternalStateLoxVent;
    bool solenoidInternalStateLngVent;
    float supplyVoltage0 = std::nanf("");
    float supplyVoltage1 = std::nanf("");
    float solenoidCurrentGn2Fill = std::nanf("");
    float solenoidCurrentGn2Vent = std::nanf("");
    float solenoidCurrentGn2Disconnect = std::nanf("");
    float solenoidCurrentMvasFill = std::nanf("");
    float solenoidCurrentMvasVent = std::nanf("");
    float solenoidCurrentMvasOpen = std::nanf("");
    float solenoidCurrentMvasClose = std::nanf("");
    float solenoidCurrentLoxVent = std::nanf("");
    float solenoidCurrentLngVent = std::nanf("");
    float temperatureLox = std::nanf("");
    float temperatureLng = std::nanf("");
    float pressureGn2 = std::nanf("");
    float pressureChamber = std::nanf("");
    uint32_t crc;
};
#pragma pack(pop)

ADC_HandleTypeDef hadc1;
SPI_HandleTypeDef hspi3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
UART_HandleTypeDef huart3;

// TC handlers
// (TC0 INOPERABLE)
// (TC1 INOPERABLE)
TcMax31855Spi tc2(&hspi3, TC2_nCS_GPIO_Port, TC2_nCS_Pin, 100);
TcMax31855Spi tc3(&hspi3, TC3_nCS_GPIO_Port, TC3_nCS_Pin, 100);
TcMax31855Spi tc4(&hspi3, TC4_nCS_GPIO_Port, TC4_nCS_Pin, 100);
TcMax31855Spi tc5(&hspi3, TC5_nCS_GPIO_Port, TC5_nCS_Pin, 100);

// solenoid internal states, these are energized state, not open or closed
int solenoidState0;
int solenoidState1;
int solenoidState2;
int solenoidState3;
int solenoidState4;
int solenoidState5;
int solenoidState6;
int solenoidState7;
int solenoidState8;

// igniter internal states
int igniterState0;
int igniterState1;

// alarm
int alarmState;

// incoming command
bool newCommand = false;
uint8_t commandBuffer[sizeof(GseCommand)];
GseCommand command;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_SPI3_Init();
    MX_USART3_UART_Init();
    MX_USB_DEVICE_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();

    HAL_GPIO_WritePin(ETH_nRST_GPIO_Port, ETH_nRST_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(TC2_nCS_GPIO_Port, TC2_nCS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TC3_nCS_GPIO_Port, TC3_nCS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TC4_nCS_GPIO_Port, TC4_nCS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TC5_nCS_GPIO_Port, TC5_nCS_Pin, GPIO_PIN_SET);

    solenoidState0 = 0;
    solenoidState1 = 0;
    solenoidState2 = 0;
    solenoidState3 = 0;
    solenoidState4 = 0;
    solenoidState5 = 0;
    solenoidState6 = 0;
    solenoidState7 = 0;
    solenoidState8 = 0;

    igniterState0 = 0;
    igniterState1 = 0;

    alarmState = 0;

    HAL_Delay(1000);

    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_Base_Start(&htim5);

    HAL_UART_Receive_IT(&huart3, commandBuffer, sizeof(GseCommand));

    uint32_t usbBufferTimer = TIM5->CNT << 16 | TIM4->CNT;

    while (1) {
        GseData data;

        uint32_t timestamp = TIM5->CNT << 16 | TIM4->CNT;
        data.timestamp = timestamp;

        // update internal states
        if (newCommand) {
            newCommand = false;

            solenoidState0 = (int)(command.solenoidStateGn2Fill);
            solenoidState1 = (int)(command.solenoidStateGn2Vent);
            solenoidState2 = (int)(command.solenoidStateGn2Disconnect);
            solenoidState3 = (int)(command.solenoidStateMvasFill);
            solenoidState4 = (int)(command.solenoidStateMvasVent);
            solenoidState5 = (int)(command.solenoidStateMvasOpen);
            solenoidState6 = (int)(command.solenoidStateMvasClose);
            solenoidState7 = (int)(command.solenoidStateLoxVent);
            solenoidState8 = (int)(command.solenoidStateLngVent);
            igniterState0 = (int)command.igniter0Fire;
            igniterState1 = (int)command.igniter1Fire;
            alarmState = (int)command.alarm;
        }

        // internal states feedback
        data.solenoidInternalStateGn2Fill = (bool)solenoidState0;
        data.solenoidInternalStateGn2Vent = (bool)solenoidState1;
        data.solenoidInternalStateGn2Disconnect = (bool)solenoidState2;
        data.solenoidInternalStateMvasFill = (bool)solenoidState3;
        data.solenoidInternalStateMvasVent = (bool)solenoidState4;
        data.solenoidInternalStateMvasOpen = (bool)solenoidState5;
        data.solenoidInternalStateMvasClose = (bool)solenoidState6;
        data.solenoidInternalStateLoxVent = (bool)solenoidState7;
        data.solenoidInternalStateLngVent = (bool)solenoidState8;
        data.igniterInternalState0 = (bool)igniterState0;
        data.igniterInternalState1 = (bool)igniterState1;
        data.alarmInternalState = (bool)alarmState;

        // switch solenoids
        HAL_GPIO_WritePin(SOLENOID0_EN_GPIO_Port, SOLENOID0_EN_Pin, (GPIO_PinState)solenoidState0);
        HAL_GPIO_WritePin(SOLENOID1_EN_GPIO_Port, SOLENOID1_EN_Pin, (GPIO_PinState)solenoidState1);
        HAL_GPIO_WritePin(SOLENOID2_EN_GPIO_Port, SOLENOID2_EN_Pin, (GPIO_PinState)solenoidState2);
        HAL_GPIO_WritePin(SOLENOID3_EN_GPIO_Port, SOLENOID3_EN_Pin, (GPIO_PinState)solenoidState3);
        HAL_GPIO_WritePin(SOLENOID4_EN_GPIO_Port, SOLENOID4_EN_Pin, (GPIO_PinState)solenoidState4);
        HAL_GPIO_WritePin(SOLENOID5_EN_GPIO_Port, SOLENOID5_EN_Pin, (GPIO_PinState)solenoidState5);
        HAL_GPIO_WritePin(SOLENOID6_EN_GPIO_Port, SOLENOID6_EN_Pin, (GPIO_PinState)solenoidState6);
        HAL_GPIO_WritePin(SOLENOID7_EN_GPIO_Port, SOLENOID7_EN_Pin, (GPIO_PinState)solenoidState7);
        HAL_GPIO_WritePin(SOLENOID8_EN_GPIO_Port, SOLENOID8_EN_Pin, (GPIO_PinState)solenoidState8);

        // igniter
        data.igniterArmed = (bool)HAL_GPIO_ReadPin(ARMED_GPIO_Port, ARMED_Pin);
        data.igniter0Continuity = !(bool)HAL_GPIO_ReadPin(EMATCH0_CONT_GPIO_Port, EMATCH0_CONT_Pin);
        data.igniter1Continuity = !(bool)HAL_GPIO_ReadPin(EMATCH1_CONT_GPIO_Port, EMATCH1_CONT_Pin);

        if (data.igniterArmed && igniterState0) {
            HAL_GPIO_WritePin(EMATCH0_FIRE_GPIO_Port, EMATCH0_FIRE_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(EMATCH0_FIRE_GPIO_Port, EMATCH0_FIRE_Pin, GPIO_PIN_RESET);
        }
        if (data.igniterArmed && igniterState1) {
            HAL_GPIO_WritePin(EMATCH1_FIRE_GPIO_Port, EMATCH1_FIRE_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(EMATCH1_FIRE_GPIO_Port, EMATCH1_FIRE_Pin, GPIO_PIN_RESET);
        }

        // alarm
        HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, (GPIO_PinState)alarmState);

        // ADC operations
        AdcData rawData = {0};
        for (int i = 0; i < 16; i++) {
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 10);
            uint32_t data = HAL_ADC_GetValue(&hadc1);
            *(((uint32_t *)&rawData) + i) += data;
        }

        data.pressureGn2 = 0.00128 * (float)rawData.pt0;                    // PT0 -> GN2 (5000 PSI)
        data.pressureChamber = (0.00128f * (float)rawData.pt1);             // PT1 ->
        (void)(0.00128f * (float)rawData.pt2);                              // UNUSED
        (void)(0.00128f * (float)rawData.pt3);                              // UNUSED
        (void)(0.00128f * (float)rawData.pt4);                              // UNUSED
        data.solenoidCurrentGn2Fill = 0.000817f * (float)rawData.s0;        // S0 -> GN2 Fill
        data.solenoidCurrentGn2Vent = 0.000817f * (float)rawData.s1;        // S1 -> GN2 Vent
        data.solenoidCurrentGn2Disconnect = 0.000817f * (float)rawData.s2;  // S2 -> GN2 QD
        data.solenoidCurrentMvasFill = 0.000817f * (float)rawData.s3;       // S3 -> MVAS Fill
        data.solenoidCurrentMvasVent = 0.000817f * (float)rawData.s4;       // S4 -> MVAS Vent
        data.solenoidCurrentMvasOpen = 0.000817f * (float)rawData.s5;       // S5 -> MVAS Open
        data.solenoidCurrentMvasClose = 0.000817f * (float)rawData.s6;      // S6 -> MVAS Close
        data.solenoidCurrentLoxVent = 0.000817f * (float)rawData.s7;        // S7 -> LOX Vent
        data.solenoidCurrentLngVent = 0.000817f * (float)rawData.s8;        // S8 -> LNG Vent
        data.supplyVoltage0 = 0.0062f * (float)rawData.pwr0 + 0.435f;
        data.supplyVoltage1 = 0.0062f * (float)rawData.pwr1 + 0.435f;

        // read thermocouples
        TcMax31855Spi::Data tcData;
        // (TC0 INOPERABLE)
        // (TC1 INOPERABLE)
        tcData = tc2.Read();
        if (tcData.valid) {
            data.temperatureLox = tcData.tcTemperature;  // TC2 -> LOX Temperature
        }
        tcData = tc3.Read();
        if (tcData.valid) {
            data.temperatureLng = tcData.tcTemperature;  // TC3 -> LNG Temperature
        }
        tcData = tc4.Read();
        if (tcData.valid) {
            (void)tcData.tcTemperature;  // UNUSED
        }
        tcData = tc5.Read();
        if (tcData.valid) {
            (void)tcData.tcTemperature;  // UNUSED
        }

        // USB
        char buffer[1024] = {0};
        if(data.timestamp - usbBufferTimer > 500)
        {
            sprintf(buffer,
                "Timestamp: %08X\r\n"
                "IGNITER\r\n"
                "Armed: %d  Igniter 0 Continuity: %d  Igniter 1 Continuity: %d\r\n"
                "(G)Igniter 1 Fire: %d  (H)Igniter 2 Fire: %d\r\n"
                "\r\n"
                "SOLENOID\r\n"
                "(1)GN2 Fill: %d-%04d  (2)GN2 Vent: %d-%04d  (3)GN2 QD: %d-%04d  (4)MVAS Fill: %d-%04d  (5)MVAS Vent: %d-%04d  "
                "(6)MVAS Open: %d-%04d  (7)MVAS Close: %d-%04d  (8)LOX Vent: %d-%04d  (9)LNG Vent: %d-%04d\r\n"
                "\r\n"
                "PRESSURE & TEMPERATURE & VOLTAGE\r\n"
                "Supply 0: %02d  Supply 1: %02d  GN2 Pressure: %04d  LOX Temperature: %03d  LNG Temperature: %03d\r\n"
                "(A)Alarm: %d\r\n"
                "---------------------\r\n",
                (unsigned int)data.timestamp, (int)data.igniterArmed, (int)data.igniter0Continuity, (int)data.igniter1Continuity, (int)data.igniterInternalState0, (int)data.igniterInternalState1,
                (int)data.solenoidInternalStateGn2Fill, (int)(data.solenoidCurrentGn2Fill * 1000), (int)data.solenoidInternalStateGn2Vent, (int)(data.solenoidCurrentGn2Vent * 1000),
                (int)data.solenoidInternalStateGn2Disconnect, (int)(data.solenoidCurrentGn2Disconnect * 1000), (int)data.solenoidInternalStateMvasFill, (int)(data.solenoidCurrentMvasFill * 1000),
                (int)data.solenoidInternalStateMvasVent, (int)(data.solenoidCurrentMvasVent * 1000), (int)data.solenoidInternalStateMvasOpen, (int)(data.solenoidCurrentMvasOpen * 1000),
                (int)data.solenoidInternalStateMvasClose, (int)(data.solenoidCurrentMvasClose * 1000), (int)data.solenoidInternalStateLoxVent, (int)(data.solenoidCurrentLoxVent * 1000),
                (int)data.solenoidInternalStateLngVent, (int)(data.solenoidCurrentLngVent * 1000), (int)data.supplyVoltage0, (int)data.supplyVoltage1, (int)(data.pressureGn2 * 1000),
                (int)data.temperatureLox, (int)data.temperatureLng, (int)data.alarmInternalState);
            CDC_Transmit_FS((uint8_t *)buffer, strlen(buffer));

            //update buffer timer
            usbBufferTimer = data.timestamp;
        }

        // ethernet
        uint32_t crc = Crc32((uint8_t *)&data, sizeof(GseData) - 4);
        data.crc = crc;
        HAL_UART_Transmit(&huart3, (uint8_t *)&data, sizeof(GseData), 100);

        // loop time control (the timestamp rolls over after 49 hours, should be ok)
        while ((TIM5->CNT << 16 | TIM4->CNT) - timestamp < 100) {
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    uint32_t crc = Crc32(commandBuffer, sizeof(GseCommand) - 4);
    if (crc == ((GseCommand *)commandBuffer)->crc) {
        memcpy((uint8_t *)&command, commandBuffer, sizeof(GseCommand));
        newCommand = true;
    }
    HAL_UART_Receive_IT(huart, commandBuffer, sizeof(GseCommand));
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }

    __HAL_RCC_PLLI2S_ENABLE();
}

static void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = ENABLE;
    hadc1.Init.NbrOfDiscConversion = 1;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 16;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_4;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = ADC_REGULAR_RANK_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = ADC_REGULAR_RANK_6;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = ADC_REGULAR_RANK_7;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_7;
    sConfig.Rank = ADC_REGULAR_RANK_8;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = ADC_REGULAR_RANK_9;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = ADC_REGULAR_RANK_10;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_10;
    sConfig.Rank = ADC_REGULAR_RANK_11;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_11;
    sConfig.Rank = ADC_REGULAR_RANK_12;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_12;
    sConfig.Rank = ADC_REGULAR_RANK_13;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_13;
    sConfig.Rank = ADC_REGULAR_RANK_14;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_14;
    sConfig.Rank = ADC_REGULAR_RANK_15;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_15;
    sConfig.Rank = ADC_REGULAR_RANK_16;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_SPI3_Init(void) {
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi3) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM4_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 36000;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM5_Init(void) {
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 65535;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
        Error_Handler();
    }
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
    sSlaveConfig.InputTrigger = TIM_TS_ITR2;
    if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART3_UART_Init(void) {
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 57600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, SOLENOID6_EN_Pin | SOLENOID7_EN_Pin | SOLENOID8_EN_Pin | EMATCH0_FIRE_Pin | EMATCH1_FIRE_Pin | TC3_nCS_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, ETH_nRST_Pin | ALARM_Pin | TC5_nCS_Pin | SOLENOID0_EN_Pin | SOLENOID1_EN_Pin | SOLENOID2_EN_Pin | SOLENOID3_EN_Pin | SOLENOID4_EN_Pin | SOLENOID5_EN_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOA, TC2_nCS_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(TC4_nCS_GPIO_Port, TC4_nCS_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = SOLENOID6_EN_Pin | SOLENOID7_EN_Pin | SOLENOID8_EN_Pin | EMATCH0_FIRE_Pin | EMATCH1_FIRE_Pin | TC3_nCS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ETH_nRST_Pin | ALARM_Pin | TC5_nCS_Pin | SOLENOID0_EN_Pin | SOLENOID1_EN_Pin | SOLENOID2_EN_Pin | SOLENOID3_EN_Pin | SOLENOID4_EN_Pin | SOLENOID5_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ARMED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ARMED_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = EMATCH0_CONT_Pin | EMATCH1_CONT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TC2_nCS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TC4_nCS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TC4_nCS_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}
