# Phase 1 SRTP部分

Time: 2024.4-2025.5

## 1. 硬件配置

### 1.1 用到的硬件

- 树莓派Raspberry Pi 4B 4GB
- 思岚激光雷达RPLIDAR A1
- L298N电机驱动模块
- MC520编码器电机
- 一个12V串联电池盒
- 面包板
- 两轮+万向轮的底盘
- 21英寸显示屏

### 1.2 没用到的硬件

- TB6612 FNG电机驱动板模块
- Arduino UNO板
- HC-SR04超声波测距模块
- LM2596S稳压模块
- N20减速马达
- 7寸显示屏
- 角码
- 电烙铁
- AB胶

## 2. 硬件组装过程

### 2.1 树莓派RPIO接口说明

[树莓派接口](https://shumeipai.nxez.com/raspberry-pi-pins-version-40)

上面的文章介绍了树莓派的GPIO引脚对照表，调用引脚的时候用python的RPi.GPIO库，设置模式为BCM，然后调用物理引脚对应的BCM编码。

e.g. 如果要调用物理引脚16，需要：

```python
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
example = 23 # 23是物理引脚16对应的BCM编码
```

这是一种简单的定义一个引脚example的方式，它的物理引脚号是16，BCM引脚号是23.

当然，标准的定义引脚方式是这样的：

```python
GPIO.setup(21, GPIO.INPUT)
GPIO.setup(23, GPIO.OUTPUT)
```

就把引脚作为GPIO输入或者输出了。

**关于GPIO引脚是否支持中断：** 所有的有BCM编号的GPIO引脚都支持中断，这和Arduino有很大区别。

### 2.2 关于编码器电机

