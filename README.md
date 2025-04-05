# 组件: PCA9557

IO扩展芯片PCA9557的I2C驱动器

基于`i2c_master.h`库来进行配置，初始化函数也只是对其I2C初始化结构的封装，您完全可以用自行初始化好的句柄来对其进行控制。

## 使用

### 初始化

> 注意：如果您的I2C总线已经初始化了，则创建设备加入即可。

```c
i2c_master_bus_handle_t bus_handle = NULL; // 创建I2C总线句柄
i2c_device_config_t     *dev_cfg   = NULL; // I2C-设备配置
i2c_master_dev_handle_t dev_handle = NULL; // I2C-设备句柄创建

bus_handle = PCA9557_I2C_CreateMasterBus(I2C_NUM_0, GPIO_NUM_1, GPIO_NUM_2);
dev_cfg    = PCA9557_I2C_CreateDeviceConfig(PCA9557_I2C_ADDR);
dev_handle = PCA9557_I2C_MasterBusAddDevice(bus_handle, dev_cfg);

// 初始化PCA9557
PCA9557_Init(dev_handle);
```



### 读写数据

> 设置扩展引脚为输出模式，并且控制它的电平。

```c
PCA9557_SetPinMode(dev_handle, PCA9557_GPIO_PIN0, PCA9557_GPIO_MODE_OUTPUT);
PCA9557_WritePin(dev_handle, PCA9557_GPIO_PIN0, PCA9557_GPIO_VALUE_LOW);
```

