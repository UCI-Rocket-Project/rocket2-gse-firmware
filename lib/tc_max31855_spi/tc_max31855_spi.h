#pragma once

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4xx)
#include "stm32f4xx_hal.h"
#endif

class TcMax31855Spi {
  public:
    typedef struct Data {
        bool valid;
        // tc temperature data in degrees celsius
        float tcTemperature;
        // reference internal temperature in degrees celsius
        float internalTemperature;
    } Data;

    /**
     * @param hspi SPI bus handler
     * @param csPort chip select GPIO port
     * @param csPin chip select GPIO pin
     * @param timeout serial bus timeout in milliseconds
     */
    TcMax31855Spi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin, uint32_t timeout);

    Data Read();

  private:
    SPI_HandleTypeDef *_hspi;
    GPIO_TypeDef *_csPort;
    uint16_t _csPin;
    uint32_t _timeout;
};
