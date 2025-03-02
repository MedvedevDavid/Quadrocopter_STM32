// #include "main.h"
// #include "i2c.h"
// #include "usart.h"
// #include "gpio.h"

// #define HMC5883L_ADDRESS 0x3C  // HMC5883L I2C address
// #define HMC5883L_REG_CONFIG_A 0x00
// #define HMC5883L_REG_CONFIG_B 0x01
// #define HMC5883L_REG_MODE 0x02
// #define HMC5883L_REG_OUT_X_M 0x03

// void HMC5883L_Init(void);
// void HMC5883L_ReadData(int16_t* x, int16_t* y, int16_t* z);

// int main(void) {
//     HAL_Init();
//     SystemClock_Config();
//     MX_GPIO_Init();
//     MX_USART2_UART_Init();
//     MX_I2C1_Init();

//     HMC5883L_Init();

//     int16_t x, y, z;

//     while (1) {
//         HMC5883L_ReadData(&x, &y, &z);

//         // Print the data to UART
//         char buffer[64];
//         snprintf(buffer, sizeof(buffer), "X: %d, Y: %d, Z: %d\r\n", x, y, z);
//         HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer),
//         HAL_MAX_DELAY);

//         HAL_Delay(1000);  // Delay 1 second
//     }
// }

// void HMC5883L_Init(void) {
//     uint8_t config[2];

//     // Set configuration register A
//     config[0] = HMC5883L_REG_CONFIG_A;
//     config[1] = 0x70;  // 8 samples averaged, 15 Hz default, normal
//     measurement HAL_I2C_Master_Transmit(&hi2c1, HMC5883L_ADDRESS, config, 2,
//     HAL_MAX_DELAY);

//     // Set configuration register B
//     config[0] = HMC5883L_REG_CONFIG_B;
//     config[1] = 0xA0;  // Gain = 5
//     HAL_I2C_Master_Transmit(&hi2c1, HMC5883L_ADDRESS, config, 2,
//     HAL_MAX_DELAY);

//     // Set mode register
//     config[0] = HMC5883L_REG_MODE;
//     config[1] = 0x00;  // Continuous measurement mode
//     HAL_I2C_Master_Transmit(&hi2c1, HMC5883L_ADDRESS, config, 2,
//     HAL_MAX_DELAY);
// }

// void HMC5883L_ReadData(int16_t* x, int16_t* y, int16_t* z) {
//     uint8_t data[6];

//     // Set the register pointer to the data output register
//     uint8_t reg = HMC5883L_REG_OUT_X_M;
//     HAL_I2C_Master_Transmit(&hi2c1, HMC5883L_ADDRESS, &reg, 1,
//     HAL_MAX_DELAY);

//     // Read data output register
//     HAL_I2C_Master_Receive(&hi2c1, HMC5883L_ADDRESS, data, 6, HAL_MAX_DELAY);

//     // Combine the MSB and LSB for each axis
//     *x = (int16_t)(data[0] << 8 | data[1]);
//     *y = (int16_t)(data[4] << 8 | data[5]);
//     *z = (int16_t)(data[2] << 8 | data[3]);
// }