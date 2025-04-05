#include <stdio.h>
#include "PCA9557.h"

// 创建I2C总线句柄（可选）
i2c_master_bus_handle_t PCA9557_I2C_CreateMasterBus(i2c_port_num_t i2c_port, 
                                                    gpio_num_t sda_io_num, 
                                                    gpio_num_t scl_io_num)
{
    // I2C-主机总线配置
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port,
        .scl_io_num = scl_io_num,
        .sda_io_num = sda_io_num,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    // I2C-主机总线句柄创建
    i2c_master_bus_handle_t bus_handle = malloc(sizeof(i2c_master_bus_handle_t));
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    return bus_handle;
}

// 创建I2C设备句柄
i2c_device_config_t *PCA9557_I2C_CreateDeviceConfig(uint8_t device_address)
{
    // I2C-设备配置
    i2c_device_config_t *dev_cfg = malloc(sizeof(i2c_device_config_t));
    dev_cfg->dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg->device_address = device_address;
    dev_cfg->scl_speed_hz = 100000;
    dev_cfg->scl_wait_us = 0;
    dev_cfg->flags.disable_ack_check = 0;

    return dev_cfg;
}

// 设备加入I2C总线
i2c_master_dev_handle_t PCA9557_I2C_MasterBusAddDevice(i2c_master_bus_handle_t bus_handle, 
                                                          i2c_device_config_t *dev_cfg)
{
    i2c_master_dev_handle_t dev_handle = malloc(sizeof(i2c_master_dev_handle_t));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, dev_cfg, &dev_handle));
    return dev_handle;
}

// 初始化PCA9557
void PCA9557_Init(i2c_master_dev_handle_t dev_handle)
{
    uint8_t send[2]  = {0}; // 发送区

    // 设置全部引脚的输出低电平
    send[0] = PCA9557_REG_OUTPUT;    // 输出寄存器
    send[1] = 0x00;                  // 设置所有引脚为输出模式
    i2c_master_transmit(dev_handle, send, sizeof(send), -1);
    // 设置全部引脚极性为不反转
    send[0] = PCA9557_REG_POLARITY;  // 输出寄存器地址
    send[1] = 0x00;                  // 设置所有引脚输出高电平
    i2c_master_transmit(dev_handle, send, sizeof(send), -1);
    // 设置全部引脚为高阻抗输入模式
    send[0] = PCA9557_REG_CONFIG;    // 输出寄存器地址
    send[1] = 0xFF;                  // 设置所有引脚为输入模式
    i2c_master_transmit(dev_handle, send, sizeof(send), -1);
}

// 读取引脚值
PCA9557_GPIO_VALUE_t PCA9557_ReadPin(i2c_master_dev_handle_t dev_handle, PCA9557_GPIO_PIN_t pin)
{
    uint8_t cmd[1]   = {0x00}; // 命令区
    uint8_t data[1]  = {0x00}; // 数据区

    // 读取引脚值
    cmd[0] = PCA9557_REG_INPUT; // 输入寄存器地址
    i2c_master_transmit_receive(dev_handle, cmd, sizeof(cmd), data, sizeof(data), -1);
    return (PCA9557_GPIO_VALUE_t)(data[0] & pin);
}

// 设置引脚值
void PCA9557_WritePin(i2c_master_dev_handle_t dev_handle, PCA9557_GPIO_PIN_t pin, PCA9557_GPIO_VALUE_t value)
{
    uint8_t cmd[1]   = {0x00}; // 命令区
    uint8_t data[1]  = {0x00}; // 数据区
    uint8_t send[2]  = {0x00}; // 发送区
    
    cmd[0] = PCA9557_REG_OUTPUT; // 输入寄存器地址
    // 读取引脚值
    i2c_master_transmit_receive(dev_handle, cmd, sizeof(cmd), data, sizeof(data), -1);
    if (value == PCA9557_GPIO_VALUE_HIGH) {
        data[0] |= pin;     // 设置引脚高电平
    } else {
        data[0] &= (~pin);  // 设置引脚低电平
    }
    // 设置引脚值
    send[0] = PCA9557_REG_OUTPUT; // 输出寄存器地址
    send[1] = data[0];            // 设置引脚值
    i2c_master_transmit(dev_handle, send, sizeof(send), -1);
}

// 设置引脚模式
void PCA9557_SetPinMode(i2c_master_dev_handle_t dev_handle, PCA9557_GPIO_PIN_t pin, PCA9557_GPIO_MODE_t mode)
{
    uint8_t cmd[1]   = {0x00}; // 命令区
    uint8_t data[1]  = {0x00}; // 数据区
    uint8_t send[2]  = {0x00}; // 发送区

    cmd[0] = PCA9557_REG_CONFIG; // 输入寄存器地址
    // 读取引脚值
    i2c_master_transmit_receive(dev_handle, cmd, sizeof(cmd), data, sizeof(data), -1);
    if (mode == PCA9557_GPIO_MODE_INPUT) {
        data[0] |= pin;     // 设置引脚为输入模式
    } else {
        data[0] &= (~pin);  // 设置引脚为输出模式
    }
    // 设置引脚值
    send[0] = PCA9557_REG_CONFIG; // 输出寄存器地址
    send[1] = data[0];            // 设置引脚值
    i2c_master_transmit(dev_handle, send, sizeof(send), -1);
}

// 设置引脚极性
void PCA9557_SetPinPolarity(i2c_master_dev_handle_t dev_handle, PCA9557_GPIO_PIN_t pin, PCA9557_GPIO_POLARITY_t polarity)
{
    uint8_t cmd[1]   = {0x00}; // 命令区
    uint8_t data[1]  = {0x00}; // 数据区
    uint8_t send[2]  = {0x00}; // 发送区

    cmd[0] = PCA9557_REG_POLARITY; // 输入寄存器地址
    // 读取引脚值
    i2c_master_transmit_receive(dev_handle, cmd, sizeof(cmd), data, sizeof(data), -1);
    if (polarity == PCA9557_GPIO_POLARITY_INVERTED) {
        data[0] |= pin;     // 设置引脚为反转极性
    } else {
        data[0] &= (~pin);  // 设置引脚为正常极性
    }
    // 设置引脚值
    send[0] = PCA9557_REG_POLARITY; // 输出寄存器地址
    send[1] = data[0];              // 设置引脚值
    i2c_master_transmit(dev_handle, send, sizeof(send), -1);
}

// 写入寄存器值
void PCA9557_WriteReg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t send[2]  = {0x00}; // 发送区

    send[0] = reg_addr; // 输出寄存器地址
    send[1] = data;     // 设置引脚值
    i2c_master_transmit(dev_handle, send, sizeof(send), -1);
}

// 读取寄存器值
uint8_t PCA9557_ReadReg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr)
{
    uint8_t cmd[1]   = {0x00}; // 命令区
    uint8_t data[1]  = {0x00}; // 数据区

    cmd[0] = reg_addr; // 输入寄存器地址
    i2c_master_transmit_receive(dev_handle, cmd, sizeof(cmd), data, sizeof(data), -1);
    return data[0];
}
