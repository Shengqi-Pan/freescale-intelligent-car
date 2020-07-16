# 硬件设计与调试

## PCB Version 3.0

电磁检测插在MCU母板上

不带5V和12V稳压的电机驱动

底盘换成PCB，把无线充电方案、5V、12V稳压和MCU母板放上去

## TODO LIST

### 母板V2.0和电磁检测V2.0板子硬件调试

- 更换左侧竖电感

- 调节四个电位器，使左右横竖电感的AD采样值对称

- 直立控制测试

- 无线串口接收测试

---------------------

**7.14~7.16：基本正常**

### 改车底盘，PCB上把5V、12V稳压、母板放在一起

- 5V和12V输出的单点接地 SGND、PGND

- 按键S2~S5、RST封装修改

- 供电开关按键修改，要能流通大电流

- 可能需要修改电机控制、编码器、陀螺仪、无线串口的接口位置

- 需要增大一些接口之间的间距，以避免排线插接冲突

- ~~TPS630701封装修改，便于焊接和虚焊检查~~

- 检查供电接入开关、母板5V开关、驱动板12V开关是否加进去了

- ~~TPS61088封装修改~~

- PCB尺寸定义与检查

- 给ICM留出2.5*1.5的空间

**7.15：车底盘，母板v4.0已制板**

### 改电磁检测PCB

- 多加一个OPA2350，增加一路电感AD采样用于判坡

- 在母板上留出的尺寸空间也要作修改

- ~~**！！！要修改滤波的方式，更改输出滤波电路，RC串联加稳压和反向限幅**~~

- ~~**直接修改OPA2350为OPA4377，增加一路电感，暂时空出一路**~~

- ~~**加一路OPA2350，仿照OPAMOD模块**~~

----------------------------;

**7.13：电磁检测v2.0已改完制板，贴片电阻改为0603封装，电容为C0805**

### 电机驱动板

----------------------------;

**7.16：焊接和调试，发现一个小问题：由于自举电容的机制约束，HIP4082驱动全桥的上管在原理上应该不能到100%占空比，这一点需要在之后的软件里面测试和注意。**

### 无线充电方案

TODO LIST...

- 恒功率充电 or 全桥整流？

### 元器件

- OPA2350

- 100K电位器

- 稳压管

- 47K电阻

- TPS630701

- 61088外围电路元件RLC

- 工字电感

--------------------------;

**7.16：买了新一批器件**