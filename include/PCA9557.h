#ifndef PCA9557_H
#define PCA9557_H

#include "driver/i2c_master.h"

/* 该库概述：
 * 提供I2C总线初始化,设备对象创建加入I2C总线
 * 提供PCA9557初始化,读写引脚值,设置引脚模式,设置引脚极性
 * 提供读写寄存器值
 */

#define PCA9557_I2C_ADDR        0x19 // PCA9557 I2C地址
#define PCA9557_REG_INPUT       0x00 // 输入寄存器
#define PCA9557_REG_OUTPUT      0x01 // 输出寄存器
#define PCA9557_REG_POLARITY    0x02 // 极性寄存器
#define PCA9557_REG_CONFIG      0x03 // 配置寄存器

// PCA9557 GPIO引脚
typedef enum{
    PCA9557_GPIO_PIN0 = 0x01, // GPIO0
    PCA9557_GPIO_PIN1 = 0x02, // GPIO1
    PCA9557_GPIO_PIN2 = 0x04, // GPIO2
    PCA9557_GPIO_PIN3 = 0x08, // GPIO3
    PCA9557_GPIO_PIN4 = 0x10, // GPIO4
    PCA9557_GPIO_PIN5 = 0x20, // GPIO5
    PCA9557_GPIO_PIN6 = 0x40, // GPIO6
    PCA9557_GPIO_PIN7 = 0x80, // GPIO7
}PCA9557_GPIO_PIN_t;

// PCA9557 GPIO值
typedef enum{
    PCA9557_GPIO_VALUE_LOW  = 0x00, // GPIO低电平
    PCA9557_GPIO_VALUE_HIGH = 0x01, // GPIO高电平
}PCA9557_GPIO_VALUE_t;

// PCA9557 GPIO模式
typedef enum{
    PCA9557_GPIO_MODE_OUTPUT = 0x00, // GPIO输出模式
    PCA9557_GPIO_MODE_INPUT  = 0x01, // GPIO输入模式
}PCA9557_GPIO_MODE_t;

// PCA9557 GPIO极性
typedef enum{
    PCA9557_GPIO_POLARITY_NORMAL   = 0x00,  // GPIO正常极性
    PCA9557_GPIO_POLARITY_INVERTED = 0x01,  // GPIO反转极性
}PCA9557_GPIO_POLARITY_t;

/***** 配置类 *****/
// 创建I2C总线句柄
i2c_master_bus_handle_t PCA9557_I2C_CreateMasterBus(i2c_port_num_t i2c_port, 
                                                    gpio_num_t sda_io_num, 
                                                    gpio_num_t scl_io_num);
// 创建I2C设备句柄
i2c_device_config_t *PCA9557_I2C_CreateDeviceConfig(uint8_t device_address);
// 设备加入I2C总线
i2c_master_dev_handle_t PCA9557_I2C_MasterBusAddDevice(i2c_master_bus_handle_t bus_handle, 
                                                           i2c_device_config_t *dev_cfg);

/***** 功能类 *****/
void PCA9557_Init(i2c_master_dev_handle_t dev_handle);

PCA9557_GPIO_VALUE_t PCA9557_ReadPin(i2c_master_dev_handle_t dev_handle, PCA9557_GPIO_PIN_t pin);
void PCA9557_WritePin(i2c_master_dev_handle_t dev_handle, PCA9557_GPIO_PIN_t pin, PCA9557_GPIO_VALUE_t value);
void PCA9557_SetPinMode(i2c_master_dev_handle_t dev_handle, PCA9557_GPIO_PIN_t pin, PCA9557_GPIO_MODE_t mode);
void PCA9557_SetPinPolarity(i2c_master_dev_handle_t dev_handle, PCA9557_GPIO_PIN_t pin, PCA9557_GPIO_POLARITY_t polarity);

void PCA9557_WriteReg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);
uint8_t PCA9557_ReadReg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr);

#endif 

