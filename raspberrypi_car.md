# Phase 1 SRTP部分

Time: 2025.4-2025.5

Progress: 小车的搭建，把激光雷达的扫描结果在rviz上显示，进行SLAM建图

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

### 1.2 买了但是没用到的硬件

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

然后再加上这个：

```python
GPIO.setup(21, GPIO.INPUT)
GPIO.setup(23, GPIO.OUTPUT)
```

就把引脚作为GPIO输入或者输出了。

**关于GPIO引脚是否支持中断：** 所有的有BCM编号的GPIO引脚都支持中断，这和Arduino有很大区别。

### 2.2 关于编码器电机

编码器电机和普通的电机不太一样，可以简单理解为普通电机给电压或者PWM信号就可以转，编码器电机给了电压或者PWM信号转了以后，会返回值给控制器。也就是说，普通电机是开环控制的，编码器电机可以实现闭环控制。

利用编码器电机的闭环控制特点就可以实现PID转速调节。

#### 2.2.1 编码器电机接线说明

如果我没记错的话，我的电机是这样接的（不同编码器电机不太一样，可以参考电机的技术文档）：

```txt
1 -- 电机电源线-    接12V电池组的负极
2 -- 编码器电源VCC  接树莓派GPIO的5V
3 -- 编码器信号A相  
4 -- 编码器信号B相  
5 -- 编码器地线GND  接树莓派GPIO的GND
6 -- 电机电源线+    接12V电池组的正极
```

A相是输出正交脉冲信号的，B相和A相相位差差了90度，差的相位可以判断旋转方向。

总之，就把A相和B相理解成电机反馈给控制器的信号就行。

#### 2.2.2 编码器电机的转速计算

我的编码器电机输入的是PWM信号，但是实际上我们要控制的是电机的转速rpm，它们之间的转换关系是：

$$
w_{rpm} = n_{pwm} / (n * PPR) * 60
$$

其中，PPR是电机每转脉冲数，在电机的技术文档里会提到。

另外需要注意， $n_{pwm}$ 并不是PWM波的频率，而是电机每秒输出的的编码器脉冲数量。

n表示的是采用的是n倍频，就是信号里不同频率成分相对于基频的倍数关系。通常信号分析的时候会有一个图谱，显示哪个频率的信号出现次数是峰值，那个就是基频，然后会有基频的倍数频率出现小一点的峰值，这就是几倍频。这个通常用在故障分析和信号排查里面，在电机这里这个东西没那么重要，n设成几都行，我用的是四倍频。

这样就建立了电机每秒脉冲数和转速rpm之间的关系。

#### 2.2.3 让电机转动

首先，先不考虑rpm和PWM占空比之间的关系，先只考虑用PWM占空比控制电机转速。占空比表示的是一个只有0和1的序列里面1占总数的百分比（注意现在说的是占空比，不是刚才说的那个PWM每秒脉冲数）。第一步你需要初始化引脚、初始化PWM，先以一个电机为例：

```python
ENA = 20  # ENA是用来给电机PWM信号的
IN1 = 5   # IN1是用来让电机正转的
IN2 = 6   # IN2是用来让电机反转的
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False) # 如果你觉得warning出现的太多很烦人可以把warning关掉

GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# 初始化pwm，设定pwm的频率是1kHz
pwm = GPIO.PWM(ENA, 1000)
pwm.start(0)
```
接下来，我们设定一个函数，函数里如果给定forward就让电机往前转，如果给定backward就让电机往后转，同时要设置好pwm的占空比：

```python
def set_motor(direction, duty_cycle)
  if direction == 'forward':
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    elif direction == 'backward':
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)

    pwm.ChangeDutyCycle(duty_cycle) # 设置pwm占空比的函数
```

接下来是主程序，用的是try-except结构（有关try-except可以看这里：[链接](https://blog.csdn.net/qdPython/article/details/121539451)）：

```python
try:
    # 让电机正转50%占空比，持续5秒
    set_motor('forward', 50)
    time.sleep(5)

    # 再反转50%占空比，持续5秒
    set_motor('backward', 50)
    time.sleep(5)

    # 停止电机
    set_motor('stop', 0)

except KeyboardInterrupt:
    pass

finally:
    pwm.stop()
    GPIO.cleanup()
```

运行这个python程序，电机就会无限时间内先正转5s，再反转5s. 如果你发现电机的转向是反的，有两种解决方法：第一种是改代码里面控制正反的HIGH和LOW，第二种是把电机驱动模块的IN1和IN2倒过来，原来接IN1的去接IN2，原来接IN2的去接IN1。

当你决定要引入rpm的时候，问题就变得复杂起来。

首先，我们有这几个数据：rpm转速值，PWM的占空比，电机每转脉冲数PPR，以及电机每秒脉冲数。

这几个数据里，电机每秒脉冲数是电机直接输出的结果，他不能输入给电机，需要输入给电机的值是PWM的占空比。所以，我们要解决的问题是：

- 怎么去建立rpm和PWM占空比之间的关系，这样我就可以输入rpm，然后变成PWM占空比的值，直接喂给电机？
- 在我喂给电机PWM占空比然后让电机转起来以后，电机反馈出来每秒脉冲数，怎么把这个数字转换成电机的实际rpm，然后输出出来？

先来看**问题2**：

$$
w_{rpm} = n_{pwm} / (4 * PPR) * 60
$$

这个关系表明了电机每秒脉冲数量和rpm之间的关系。根据我的技术文档，PPR=1560，这个会根据电机的不同而有所不同。

我们需要的是，电机已经转起来了，它一定会输出一个电机每秒脉冲数，我就可以把他转化成rpm然后输出出来。

我们需要先获取电机每秒的脉冲数量：

```python
def get_pulse_freq(self):
    now = time.time()
    dt = now - self.last_time
    freq = self.count / dt  # Hz
    self.count = 0
    self.last_time = now
    return freq
```

有关self可以看这里：[self](https://blog.csdn.net/luanfenlian0992/article/details/105146518)

这篇文章有点复杂，可能会看不懂，举个例子你就明白了：

```python
class Dog:
    def __init__(self, name):
        self.name = name
    def bark(self):
        print(f"{self.name}: Woof!")
```

你输入了一个name给self.name，那么之后你就可以在任何地方调用self.name作为name的值了，哪怕是在这个class外面，只要有self，都可以使用。

有点说多了，刚才我们获取了电机每秒的脉冲数量，接下来我们编写从每秒脉冲数到rpm的转换关系函数：

```python
def pwm_to_rpm(n_pwm, ppr):
    return (n_pwm / (4 * ppr) * 60)
```

编写好以后，在主程序里：

```python
PPR = 1560
n_pwm = encoder.get_pulse_freq()
rpm = pwm_to_rpm(n_pwm, PPR)
```

就得到了电机转动的时候电机rpm的实时值。

再来看**问题1**：

理论上讲，rpm和PWM占空比之间是没有一个公式能表达的关系的，但你可以近似理解为y=kx+b这种线性的关系。那么我们可以在PWM占空比从0到100的范围内取几个点，然后记录这几个占空比情况下的rpm值，再直线拟合，就得到了rpm和PWM占空比之间的关系。

假如说我让PWM的占空比为20, 30, 40, 50, 60, 70, 80, 90, 100（占空比太低电机在死区不会转），用刚才获取rpm的方法，就可以得到一组数据：

```python
pwms = np.array([20, 30, 40, 50, 60, 70, 80, 90, 100])
rpms = np.array([10, 22, 35, 48, 60, 71, 79, 85, 90]) # 假如说测出来的结果是这样的
```

然后拟合一个rpm和PWM占空比的一次函数 $rpm = a * pwm + b$：

```python
a, b = np.polyfit(pwms, rpms, 1)
```

然后建立rpm和PWM占空比之间的关系：

```python
def rpm_to_pwm_linear(rpm_target):
    pwm = (rpm_target - b) / a
    pwm = max(0, min(100, pwm))
    return pwm
```

**注意：** 这里的pwm占空比一定要在0到100之间，否则会报错。

然后就可以输入一个你想要的rpm值，去直接控制PWM的占空比了：

```python
target_rpm = 60
pwm_duty = rpm_to_pwm_linear(target_rpm)
pwm.ChangeDutyCycle(pwm_duty)
```

这样就做到了给一个我想要的转速，然后让电机转起来的效果。

---

需要介绍一下python的thread库。如果说没有thread的话，我们的程序是这样的：

```python
while(a == 1)
{
    A
    B
    C
}
```

这个代码在每一次的while循环里，都只能先运行A，A运行完再运行B，以此类推。

如果说A这个语句运行的时间特别长，我又想让A运行的时候同时运行后面的B，那么就要用到thread:

```python
A_thread = threading.Thread(target=A, daemon=True)
A_thread.start()
```

daemon=True表示主程序结束时，这个thread也会结束。

以下是截止到现在的完整代码，用来给定一个电机转速rpm，然后让电机转动：

```python
import RPi.GPIO as GPIO
import time
import threading
import numpy as np

ENA = 20
IN1 = 5
IN2 = 6
ENCODER_A = 2
ENCODER_B = 3

PPR = 1560

class Encoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.count = 0
        self.last_time = time.time()
        self.lock = threading.Lock() # 创建一个锁，后续程序运行要先调用锁(with)才能运行，避免一个变量同时被多个地方改掉
    
        GPIO.setup(self.pin_a, GPIO.IN)
        GPIO.setup(self.pin_b, GPIO.IN)
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._increment)

    def _increment(self, channel): # 用来计算count
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        direction = 1 if a != b else -1
        with self.lock:
            self.count += direction

    def get_pulse_freq(self): # 用来根据count计算电机每秒脉冲数
        with self.lock:
            now = time.time()
            dt = now - self.last_time
            freq = self.count / dt  # Hz
            self.count = 0
            self.last_time = now
        return freq

class Motor:
    def __init__(self, ena, in1, in2):
        self.ena = ena
        self.in1 = in1
        self.in2 = in2
        GPIO.setup(self.ena, GPIO.OUT)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        self.pwm = GPIO.PWM(self.ena, 1000)
        self.pwm.start(0)

    def set(self, direction, duty_cycle):
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(duty_cycle)

    def stop(self):
        self.set('stop', 0)
        self.pwm.stop()

def pwm_to_rpm(n_pwm, ppr):
    return (n_pwm / (4 * ppr)) * 60

def rpm_to_pwm_linear(rpm_target):
    pwm = (rpm_target - b) / a
    pwm = max(0, min(100, pwm))
    return pwm

def encoder_thread_func(): # 读取电机的每秒脉冲数并转换成rpm
    while True:
        freq = encoder.get_pulse_freq()
        rpm = pwm_to_rpm(freq, PPR)
        time.sleep(0.5)

# 初始化
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
motor = Motor(ENA, IN1, IN2)
encoder = Encoder(ENCODER_A, ENCODER_B)

pwms = np.array([20, 30, 40, 50, 60, 70, 80, 90, 100])
rpms = np.array([10, 22, 35, 48, 60, 71, 79, 85, 90])
a, b = np.polyfit(pwms, rpms, 1)

encoder_thread = threading.Thread(target=encoder_thread_func, daemon=True)
encoder_thread.start()

try:
    while True:
        target_rpm = float(input("输入目标转速rpm: "))
        pwm_duty = rpm_to_pwm_linear(target_rpm)
        motor.set('forward', pwm_duty)
        print(f"目标RPM: {target_rpm}, 对应PWM占空比: {pwm_duty:.2f}%")

except KeyboardInterrupt:
    print("程序终止")

finally:
    motor.stop()
    GPIO.cleanup()
```

慢慢看这个程序，应该还是能看懂的。ps: 你不需要知道这里面的具体逻辑，只需要知道每一块是做什么的，逻辑有ai帮你写。

#### 2.2.4 编码器电机的PID转速控制

**关于PID：** Kp, Ki, Kd这三个参数会比较难以理解，我来举个生活中的例子：

假如说你要洗澡，用的是那种可以手动左右转然后调节洗澡水温度的出水开关。

Kp代表的是你每次调节这个开关的力度大小，Kp小你用的力就小，一次调节的温度就小；Kp大转动的就多，温度变化就会很大。

Ki代表的是如果说水温和你期待的不一样，假如说比你期待的水温低，那么你从你感受到这个低温的水开始，到你决定把开关调热一点的反应时间。Ki小代表着你反应快，Ki大就反应慢。

Kd代表的是你把水温往你理想水温调的时候的保守程度，Kd越大，你越要调到期待的水温的时候调的就会更精细，也就会更保守。

我感觉Kp和Kd应该都比较好理解，但Ki可能和你正常理解是完全反过来的。为什么Ki越大，反应反而越慢呢？因为Ki是积分项，积分是需要一段时间的，就像下雨的时候你拿个盆接水一样，Ki大，你这个盆能盛的水就多，水溢出来需要的时间就长。这里面就是Ki大，那么水温不对的时候你就在那积累，比如每1s你感觉差了5度，那么给sum加上5，当sum到最大值的时候，就是你调节水温的时候，所以Ki大你两次调节水温之间的时间就长，就反应慢。

现在你已经掌握了Kp, Ki, Kd的概念了，接下来是怎么调这三个值：

这里有两种经典情况：

- 有静差（最终稳定的值和理想的稳定值有很大差别）：这是Kp太小造成的。Kp代表的是你拧开关的力度，Kp小的时候，原来差20度，你用很小的力，相当于动了3度；现在差5度，你还是用很小的力，但这次就动了1度。总之，你的力会越来越小，最终也绝对到不了平衡点。
- 震荡：这个原因就很多了，首先是Kp太大，因为你每次都会使劲拧开关，一次往高温拧一次往低温拧，会导致温度来来回回的变；另外就是Ki太大，因为Ki大相当于你一次把之前积攒的和理想温度不符的温度全都倾泻在了这个开关上，温度变化一定很猛烈；还有可能是Kd太小，Kd太小就意味着你的调节不保守，那么波形就会来回震荡。

一般调节的时候是先调Kp，先让稳定值和你想要的稳定值相等（不需要管震荡），Kp的值常见的从几到几十都有，甚至几百都有可能；Kp调好调Ki，拿Ki去限制震荡，但Ki的值不会太大，一般都在1以下（有的时候会用Ti，Ti就是Ki的倒数）；调Ki的时候如果始终不满意就动动Kd，但Kd通常会非常非常小，常见的就是0和0.0001之类的，并且你调了就会发现Kd的影响不是那么大，主要还是Kp和Ki的影响。

好的，现在你已经有PID的理论基础了，接下来进入实战：

首先你需要清楚的是，当你引入PID的时候，你不需要在代码里表明PWM占空比与电机转速的显式关系了。

**为什么呢？**

因为PID可以自己调节，假如说现在电机的转速小于理想的电机转速，那么不管现在的占空比是多少，它都会调大占空比，而这个占空比我们不需要知道。所以，我们只需要关注电机的转速rpm就可以了。

或者你可以理解为，如果有output=pid(rpm)，那么PID会自己去适配占空比，这个输出的output也是PWM占空比的值。

（也就是说，加入了PID以后，整体的逻辑反而会简单一些）

利用python的simple_pid库可以实现转速调节，以一个电机为例，需要创建一个和PID有关的函数，比如这样：

```python
def pid_control_loop(motor, encoder, pid):
    while True:
        current_rpm = encoder.current_rpm
        with lock:
            pid.setpoint = target_rpm
        
        output = pid(current_rpm)
        pwm_duty = max(0, min(100, output))
        direction = 'forward' if output >= 0 else 'backward'
        motor.set(direction, abs(pwm_duty))
        
        print(f"Target: {target_rpm:.1f} RPM | Current: {current_rpm:.1f} RPM | PWM: {pwm_duty:.1f}%")
        time.sleep(0.1)
```

这样就实现了给定目标转速target_rpm, 实际转速current_rpm的时候，PID会自己计算出占空比output的功能。

接下来需要再把完整程序放上来一次，在这次里，我们加入了__main__函数，也就是整个程序的主函数，让代码更容易维护：

```python
import RPi.GPIO as GPIO
import time
import threading
import numpy as np
from simple_pid import PID

ENA = 20
IN1 = 5
IN2 = 6
ENCODER_A = 2
ENCODER_B = 3
PPR = 1560

# ------------------------- 编码器类 -------------------------
class Encoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.count = 0
        self.current_rpm = 0.0
        self.last_time = time.time()
        self.lock = threading.Lock()
        
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._increment)

    def _increment(self, channel):
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        direction = 1 if a != b else -1
        with self.lock:
            self.count += direction

    def get_pulse_freq(self):
        with self.lock:
            now = time.time()
            dt = now - self.last_time
            freq = self.count / dt if dt != 0 else 0
            self.count = 0
            self.last_time = now
        return freq

    def update_rpm(self):
        while True:
            freq = self.get_pulse_freq()
            self.current_rpm = (freq / (4 * PPR)) * 60
            time.sleep(0.1)

# ------------------------- 电机类 -------------------------
class Motor:
    def __init__(self, ena, in1, in2):
        self.ena = ena
        self.in1 = in1
        self.in2 = in2
        GPIO.setup([self.ena, self.in1, self.in2], GPIO.OUT)
        self.pwm = GPIO.PWM(self.ena, 1000)
        self.pwm.start(0)

    def set(self, direction, duty_cycle):
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(duty_cycle)

    def stop(self):
        self.set('stop', 0)
        self.pwm.stop()

# ------------------------- 全局变量 -------------------------
target_rpm = 0.0
lock = threading.Lock()

# ------------------------- 线程函数 -------------------------
def pid_control_loop(motor, encoder, pid):
    while True:
        current_rpm = encoder.current_rpm
        with lock:
            pid.setpoint = target_rpm
        
        output = pid(current_rpm)
        pwm_duty = max(0, min(100, output))
        direction = 'forward' if output >= 0 else 'backward'
        motor.set(direction, abs(pwm_duty))
        
        print(f"Target: {target_rpm:.1f} RPM | Current: {current_rpm:.1f} RPM | PWM: {pwm_duty:.1f}%")
        time.sleep(0.1)

def input_thread():
    global target_rpm
    while True:
        try:
            new_target = float(input("输入目标转速rpm: "))
            with lock:
                target_rpm = new_target
        except ValueError:
            print("请输入有效数字")

# ------------------------- 主程序 -------------------------
if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    
    motor = Motor(ENA, IN1, IN2)
    encoder = Encoder(ENCODER_A, ENCODER_B)
    
    pid = PID(
        Kp=1.0,
        Ki=0.1,
        Kd=0.05,
        setpoint=0,
        output_limits=(-100, 100),
        sample_time=0.1
    )

    threading.Thread(target=input_thread, daemon=True).start()
    threading.Thread(target=encoder.update_rpm, daemon=True).start()
    threading.Thread(target=pid_control_loop, args=(motor, encoder, pid), daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("程序终止")
    finally:
        motor.stop()
        GPIO.cleanup()
```

在全局变量里修改target_rpm就可以修改目标转速了。

当然，这里的输出都是“Target_RPM | Current_RPM | PWM”的形式，并没有画转速随着时间的变化图像，所以看稳定性会稍微麻烦一些（但是也要记得调PID哈）。但是毕竟是电机，转速不必要求的那么严格，差不多稳定了就好。

#### 2.2.5 通过键盘控制电机运动

刚才所有的代码都是只控制一个电机的转速，如果要用键盘控制电机转动，需要代码里控制两个电机motor_left和motor_right. 所以原来代码里有motor和encoder部分都需要改成两个电机或者两个编码器。

因为键盘需要输入wasd，因此没办法输入电机的rpm了，这里默认rpm=30. wasd的控制逻辑如下：

```text
w: rpm_left = 30    rpm_right = 30
s: rpm_left = -30   rpm_right = -30
a: rpm_left = -30   rpm_right = 30
d: rpm_left = 30    rpm_right = -30
w & a: rpm_left = 0    rpm_right = 30
w & d: rpm_left = 30   rpm_right = 0
```

s&a和s&d没做，我实在想不出来这两种情况小车要怎么动。总之，wasd的控制代码是这样的：

```python
def input_thread():
    global left_target_rpm, right_target_rpm
    base_rpm = 30
    
    while True:
        w = keyboard.is_pressed('w')
        a = keyboard.is_pressed('a')
        s = keyboard.is_pressed('s')
        d = keyboard.is_pressed('d')
        
        left_rpm = 0.0
        right_rpm = 0.0
        
        if w：
            left_rpm = base_rpm
            right_rpm = base_rpm
        elif s:
            left_rpm = -base_rpm
            right_rpm = -base_rpm
        elif a:
            left_rpm = -base_rpm
            right_rpm = base_rpm
        elif d:
            left_rpm = base_rpm
            right_rpm = -base_rpm
        elif w and a:
            left_rpm = 0
            right_rpm = base_rpm
        elif w and d:
            left_rpm = base_rpm
            right_rpm = 0
        else:
            left_rpm = 0
            right_rpm = 0
        
        with lock:
            left_target_rpm = left_rpm
            right_target_rpm = right_rpm
        
        time.sleep(0.1)
```

---

看来还得再放一次完整代码：

```python
import RPi.GPIO as GPIO
import time
import threading
import keyboard
from simple_pid import PID

ENA = 20
ENB = 21
ENCODER1_A = 3
ENCODER1_B = 2
ENCODER2_A = 17
ENCODER2_B = 27
IN1 = 5
IN2 = 6
IN3 = 13
IN4 = 19

PPR = 1560  # 编码器每转脉冲数

# ------------------------- 编码器类 -------------------------
class Encoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.count = 0
        self.current_rpm = 0.0
        self.last_time = time.time()
        self.lock = threading.Lock()
        
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._increment)

    def _increment(self, channel):
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        direction = 1 if a != b else -1
        with self.lock:
            self.count += direction

    def get_pulse_freq(self):
        with self.lock:
            now = time.time()
            dt = now - self.last_time
            freq = self.count / dt if dt != 0 else 0
            self.count = 0
            self.last_time = now
        return freq

    def update_rpm(self):
        while True:
            freq = self.get_pulse_freq()
            self.current_rpm = (freq / (4 * PPR)) * 60
            time.sleep(0.1)

# ------------------------- 电机类 -------------------------
class Motor:
    def __init__(self, ena, in1, in2):
        self.ena = ena
        self.in1 = in1
        self.in2 = in2
        GPIO.setup([self.ena, self.in1, self.in2], GPIO.OUT)
        self.pwm = GPIO.PWM(self.ena, 1000)
        self.pwm.start(0)

    def set(self, direction, duty_cycle):
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(duty_cycle)

    def stop(self):
        self.set('stop', 0)
        self.pwm.stop()

# ------------------------- 全局变量 -------------------------
left_target_rpm = 30
right_target_rpm = 30
lock = threading.Lock()

# ------------------------- 控制线程 -------------------------
def pid_control_loop(motor, encoder, pid, is_left):
    while True:
        current_rpm = encoder.current_rpm
        with lock:
            target = left_target_rpm if is_left else right_target_rpm
            pid.setpoint = target
        
        output = pid(current_rpm)
        pwm_duty = max(0, min(100, output))
        direction = 'forward' if output >= 0 else 'backward'
        motor.set(direction, abs(pwm_duty))
        
        print(f"{'Left' if is_left else 'Right'} Target: {target:.1f} RPM | Current: {current_rpm:.1f} RPM | PWM: {pwm_duty:.1f}%")
        time.sleep(0.1)

def input_thread():
    global left_target_rpm, right_target_rpm
    base_rpm = 30
    
    while True:
        w = keyboard.is_pressed('w')
        a = keyboard.is_pressed('a')
        s = keyboard.is_pressed('s')
        d = keyboard.is_pressed('d')
        
        left_rpm = 0.0
        right_rpm = 0.0
        
        if w:
            left_rpm = base_rpm
            right_rpm = base_rpm
        elif s:
            left_rpm = -base_rpm
            right_rpm = -base_rpm
        elif a:
            left_rpm = -base_rpm
            right_rpm = base_rpm
        elif d:
            left_rpm = base_rpm
            right_rpm = -base_rpm
        elif w and a:
            left_rpm = 0
            right_rpm = base_rpm
        elif w and d:
            left_rpm = base_rpm
            right_rpm = 0
        else:
            left_rpm = 0
            right_rpm = 0
        
        with lock:
            left_target_rpm = left_rpm
            right_target_rpm = right_rpm
        
        time.sleep(0.1)

# ------------------------- 主程序 -------------------------
if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    
    # 初始化左右电机和编码器
    motor_left = Motor(ENA, IN1, IN2)
    motor_right = Motor(ENB, IN3, IN4)
    encoder_left = Encoder(ENCODER1_A, ENCODER1_B)
    encoder_right = Encoder(ENCODER2_A, ENCODER2_B)

    # PID参数配置
    pid_left = PID(1.0, 0.1, 0.05, setpoint=0, output_limits=(-100, 100))
    pid_right = PID(1.0, 0.1, 0.05, setpoint=0, output_limits=(-100, 100))
    
    # 启动线程
    threading.Thread(target=input_thread, daemon=True).start()
    threading.Thread(target=encoder_left.update_rpm, daemon=True).start()
    threading.Thread(target=encoder_right.update_rpm, daemon=True).start()
    threading.Thread(target=pid_control_loop, args=(motor_left, encoder_left, pid_left, True), daemon=True).start()
    threading.Thread(target=pid_control_loop, args=(motor_right, encoder_right, pid_right, False), daemon=True).start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("程序终止")
    finally:
        motor_left.stop()
        motor_right.stop()
        GPIO.cleanup()
```

这样就可以用键盘的wasd控制小车的移动了。

### 2.3 激光雷达

我用的是RPLIDAR A1, 这是一个2D的激光雷达，所以slam建图建出的结果也只能是2D平面的图，或者你可以理解为和做CT一样在一个特定平面扫描出来的结果。这个是有很大弊端的，因为对于一些在不同高度形状不同的障碍物，可能识别会有问题，在地面不平的时候更是会出现大问题（但没办法，没钱买更好的激光雷达了，我还想买个深度相机呢，奈何是真的没钱）。

如果有人资金充裕的话，可以考虑从3D的雷达，这样也可以省下买深度相机的钱。

拿到激光雷达之后，他会自带一个芯片，按照激光雷达的技术手册把芯片连接好，然后直接连到树莓派上就可以。这个是用USB口连接的，不需要考虑GPIO的接口。连上以后激光雷达就会开始转，其实就是已经开始运行了。

### 2.4 电机驱动模块

用的是L298N的电机驱动模块，当然你也可以用TB6612等电机驱动模块。我用L298N是因为L298N我之前用过，比较熟练，但是L298N有一个大大的散热片，电机如果运转时间很长的话，有可能会烫到手（或者烫到导线）。

#### 2.4.1 L298N的接线方式

[L298N](https://blog.csdn.net/GuanFuXinCSDN/article/details/104158512)

上面的文章介绍了L298N的具体内容，按照文章里的方法1来接线。具体接线是这样的：

```text
IN1 - 电机1电源线+
IN2 - 电机1电源线-
IN3 - 电机2电源线+
IN4 - 电机2电源线-
12V - 12V电池正极
GND - 12V电池负极和树莓派GND引脚
5V - 树莓派5V引脚
```

#### 2.4.2 为什么要用电机驱动模块而不是直接连接电机？

如果电源直接连接电机，那么电机接收到的就是一个固定的电压和电流，电机只能以一个恒定的速度和恒定的方向转动。电机驱动模块可以控制输出的电压电流，从而控制电机的转速。

另外，如果电源直接连接电机的话，电源提供的电流(<50mA)不足以直接驱动电机，驱动模块有放大电流的电路。

### 2.5 超声波模块

#### 2.5.1 代码实现
HC-SR04是最经典的超声波测距模块了，网上有很成熟的教程：[HC-SR04](https://blog.csdn.net/ling3ye/article/details/51407328)

当然这个教程是连接Arduino的，里面的代码也是arduino语言，这个需要换成在树莓派里可以运行的python代码：

```python
TRIG = 14
ECHO = 15 #初始化引脚，取决于你把超声波的trig和echo接在了哪个引脚上

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

while(1)
{
  GPIO.output(TRIG, True)
  time.sleep(0.00001)
  GPIO.output(TRIG, False)
  start_time = time.time()
  stop_time = time.time()
  while GPIO.input(ECHO) == 0:
    start_time = time.time()
  while GPIO.input(ECHO) == 1:
    stop_time = time.time()
  delta_time = stop_time - start_time
  distance = (delta_time * 34000） / 2 # 声速340m/s
  time.sleep(0.1)
}
```

这个代码的逻辑很明确，就是发送记一个时间，接收记一个时间，然后用声速计算距离，输出的distance就是测量到的距离。

#### 2.5.2 超声波测距的原理

这个传感器有两个大筒，一个是发射超声波的，一个是接收超声波的，用很经典的 $v=s/t$ 就能测出来距离。

但也有很明显的缺点，他只能测正前方以及最多15度的范围，如果说要进行侧面避障和后面避障，要么用其他的传感器，要么安一圈HC-SR04。

（为什么有这个缺点还要用呢？因为简单还便宜。）

当然，超声波的精确度其实完全不行，我之前试过用只带超声波的一个别人的AGV来SLAM建图，建完的图我只能说，和实际场景差的太多，完全对不上。

精确度方面，如果只考虑避障的话，用红外传感器会更好一点。如果是为了建图，还是用雷达和深度相机这种先进的更好一些。

### 2.6 关于显示屏

一开始我是买了个7寸的显示屏，想着就简单显示一个ubuntu的桌面就可以了。但是rviz（如果你不知道的话，你先理解为一个电脑上的应用）要显示的东西比较多，7寸的显示屏分辨率太小，放不下屏幕上的所有信息，还得疯狂的拖动窗口才能看到一部分。

最后我就换了一个21寸的显示器，这个确实显示效果不错，现在还在我的桌子上当电脑的显示器。

### 2.7 整个AGV的硬件连接

#### 2.7.1 GPIO接口

```text
GPIO物理编码 | GPIO功能编码 | BCM编号 | 接口
2              -        5V输出接面包板
4              -        5V输出接树莓派散热风扇
6              -        GND输出接树莓派散热风扇
14             -        GND输出接面包板
16             23       超声波传感器Trig
18             24       超声波传感器Echo
38             20       电机驱动模块ENA
40             21       电机驱动模块ENB
3              2        电机1信号B相
5              3        电机1信号A相
11             17       电机2信号A相
13             27       电机2信号B相
29             5        电机驱动模块IN1
31             6        电机驱动模块IN2
33             13       电机驱动模块IN3
35             19       电机驱动模块IN4
```

其他需要5V连接的直接连接面包板5V，需要GND连接的直接连接面包板GND。

#### 2.7.2 其他线路连接

- RPLIDAR A1的芯片直接通过USB口连接到树莓派上面；
- 显示器的线路是HDMI-micro HDMI转接线，其中HDMI端接显示器，micro HDMI端接树莓派。
- 键盘和鼠标要用USB的，直接接在树莓派上，蓝牙的不行。
- 树莓派直接用c口接220V电源，同时显示器也要接220V电源。显示器的电源线**不要接树莓派**，因为电压不够。

**注意：** micro HDMI的头长的非常像A口，但是实际上不一样，A口插不进去树莓派。

## 3. 树莓派基础环境配置

### 3.1 系统烧录

首先，你要区分桌面版和服务器版的系统。

**ubuntu**，也就是标准的桌面版的ubuntu，它有图形界面，更适配鼠标操作；
**ubuntu server**：这个是服务器版，没有图形界面，你只能输入代码来运行系统的程序。

最开始我是没有买显示屏的，所以我觉得装个ubuntu server就行了。但是后来发现系统第一次启动的时候需要在程序里面输账号密码才可以联网然后ssh远程连接，于是我不得不买了个显示器。那既然显示器都买了，系统我也懒得换，那就给server的系统安个图形界面吧。

于是我绕了最远的路，然后就只安了一个ubuntu server.

然后，你需要了解一下计算机的处理器架构：

- x86: 最原始的计算机处理器架构，32位的，现在基本没人用了，现在用的都是他的升级版。
- armhf: ARM公司生产的32位的处理器架构，当然现在32位已经很少见了。ARM公司他不像AMD那样直接生产芯片，他会把自己的技术授权给别的厂商，然后别的厂商来生产芯片。
- arm64: ARM的64位芯片，这个现在很多都在用，比如说手机的很多芯片。如果你知道苹果的M芯片的话，M芯片也是arm64的架构。另外，树莓派也是arm64的架构。
- amd64: AMD公司生产的64位芯片，这个大多都用在电脑芯片上。比如AMD自己的芯片、intel的芯片等等。

你还需要知道ubuntu mate:

ubnutu mate就是一个为了让ubuntu系统能够稳定运行而单独建立的系统。他能在一些较老的设备上运行，ubuntu不可以（也不是说不可以，可能会卡，可能会崩，反正就这个意思）。

你还需要知道iso和img：

这两个都是镜像文件，iso是光盘镜像文件，电脑读取了iso会认为这东西是插了光盘然后读出来的东西（或者你有时候会看到显示的是DVD驱动器）。而img是普通的镜像文件，但是网上的img包都不会以img的形式出现，而是它的压缩包形式img.xz. 

对，img的压缩版本不是什么img.zip或者img.rar, 而是img.xz.

好了，当年什么都不懂的我准备了一大堆的镜像文件：

```text
#1  ubuntu-16.04.7-desktop-amd64.iso
#2  ubuntu-18.04.5-preinstalled-server-arm64+raspi4.img.xz
#3  ubuntu-18.04.6-desktop-amd64.iso
#4  ubuntu-20.04.5-preinstalled-server-arm64+raspi4.img.xz
#5  ubuntu-20.04.5-preinstalled-server-armhf+raspi4.img.xz
#6  ubuntu-20.04.6-desktop-amd64.iso
#7  ubuntu-22.04.4-desktop-amd64.iso
#8  ubuntu-mate-18.04.5-desktop-amd64.iso
#9  ubuntu-mate-20.04.1-desktop-arm64+raspi4.img.xz
#10 ubuntu-mate-20.04-beta1-desktop-arm64+raspi4.img,xz
```

已知我的树莓派4B的系统需要满足以下条件才能运行：

- ubuntu版本18或者20，太低的太老旧了打不开，太高的树莓派发布的时候ubuntu还没发布，肯定运行不了。
- 树莓派4B是arm架构的，所以只能用arm架构不能用amd架构。
- 后续安装ROS的时候要求要用64位的系统，不然会有很多包不支持。
- 我的树莓派是4GB RAM，性能还是比较强的，所以用不着用mate版本的ubuntu。
- 树莓派只支持img的镜像文件，iso的它不认识。

那么就只有4号选手适合这份工作了，我们拿出4号选手，准备进行烧录。

烧录其实并不难，只需要把树莓派的c口连接到电脑的USB，然后下载一个树莓派官方的烧录软件raspberry pi imager，再把你树莓派自带的存储卡和电脑连接好。

打开raspberry pi imager以后，先选择树莓派设备(raspberry pi 4)，然后再选择一个烧录的系统，再选择刚才的SD卡。

选择系统的时候，他会有树莓派官方的系统Raspberry Pi OS, 这个我没试过，但是听别人说挺好用的，好像是ubuntu改的系统，感兴趣的话可以试试。

当然，ubuntu的系统可扩展性好一点，所以我还是选择ubuntu. 选系统的时候选use custom, 然后找到你自己的电脑里的那个img镜像就可以了。

然后先别烧录，点了next以后他会有一个是否保持默认设置的弹窗，先别点，看下一步。

### 3.2 wifi和ssh配置

如果你不知道什么是ssh看这里：[ssh](https://blog.csdn.net/li528405176/article/details/82810342)

说白了，ssh就是能让你在windows的编辑器里查看和编辑树莓派里的文件的工具，就和远程操控一样。

好了，刚才说有一个是否保持默认设置的弹窗，这个时候可以点击advances options（应该是叫这个，我不记得了，总之不保持默认就对了），进入一个新的页面。

找到ssh的选项，把它打开。

找到wifi的选项，添加你让树莓派启动以后想让树莓派自动连接的wifi的账号和密码。注意，最好是没有用户名的wifi（校网是有用户名的），如果说你周围找不到这样的wifi，就开手机热点吧，只不过会比较费流量。

现在一切设置完毕，可以开始烧录系统了。

如果树莓派启动的时候连不上wifi, 或者后面ssh有问题，可以看看下面。

**那如果我没设置ssh和wifi怎么办？**

等系统烧录完以后，在你的电脑里打开这张存储卡所在的页面，添加一个叫ssh的文件（ssh前面没有点，就叫ssh），没有内容，放在那就行。

另外，再添加一个叫wpa_supplicant.conf的文件，内容如下：

```text
country=CN
update_config=1
network={
ssid="wifi-name"
psk="password"
priority=1
}
```

把wifi-name改成wifi名字（双引号留下），password改成wifi密码就可以了。

如果你想加好多个wifi, 让树莓派找不到第一个wifi的时候再去找第二个，你可以再新加一个network，把信息写好，然后设一下priority. 这个priority是数字越大越先搜索，也就是说priority=10是要比priority=1先搜索的。

另外，你也可以提前设好ubuntu系统的用户名和密码，好像也是在那个界面里。但这步其实没什么作用（因为第一次启动树莓派一定会让你改密码），如果你想改树莓派的用户名的话可以尝试一下（默认的绝大多数是ubuntu，也有叫pi的，但是很少）。

### 3.3 第一次启动树莓派

通电之前，记得给树莓派接上风扇，风扇接线在前面GPIO接口那里有，其实就是一个接5V，一个接GND，你直接把风扇接在一个5V电源上也不是不行。让风扇直吹树莓派上那个灰色的芯片就可以了。

如果不装风扇的话，短时间运行树莓派没什么问题，但长时间运行会让那颗芯片非常的烫。

ok，我们来启动树莓派。先把树莓派连接到显示屏、键盘和鼠标，然后给树莓派通电。正常情况下显示屏会疯狂的蹦出一堆字来，等着他蹦就可以。

**但有一种特殊情况：** 如果你看到屏幕上有树莓派的logo, 然后树莓派上面的绿灯长闪四下短闪四下，那么你烧系统烧成了树莓派不支持的，需要重新找一个镜像重新烧录进去。

我们回到蹦字的部分，它运行一会以后会停，但也没说让你输用户名和密码，这个时候你直接输入系统的用户名就可以（一般默认的都是ubuntu或者pi），然后他就会让你输入密码了（默认的都是ubuntu）。

成功之后，他会要求你改密码，并且不能改太简单的，比如1234就不行。

然后你就会进入一个充满了代码的系统里面，你可以输入命令来操作这个系统了，比如：

```text
sudo apt update
sudo apt upgrade
```

第一个代码是用来刷新整个系统的文件的，第二个代码是升级文件的，当系统内的东西有变动的时候就可以跑一跑这两个代码，说不定有的时候就是忘了它们两个然后导致报错了。

apt也可以换成apt-get. apt是apt-get的一个封装版本，当你调用apt的时候，其实底层还是在运行apt-get.

sudo是临时赋予最高权限的意思。

正常情况下，你的树莓派应该已经联网了，你可以试试下面的代码看看是不是已经成功联网了：

```text
ping www.baidu.com
```

他会发送4次信息给baidu.com的服务器，然后服务器返回信息给你自己的电脑，如果有来自什么什么的信息，就证明你的网络没有问题。

（顺便提一句，这个指令用来检测网络挺好用了，我室友之前被隔壁网络屏蔽器屏蔽了，师傅来解决的时候就一直在ping. 另外，你应该能懂在国内ping google是在做什么）

如果成功联网了，我们来配置ssh：

首先，确保你的电脑和树莓派连接到了同一个wifi，不要电脑连着校网，树莓派连着你的手机热点。

然后，在树莓派上，获取树莓派的ip地址：

```text
hostname -I
```

记住这个ip地址，一会连接ssh的时候会有用。

在你自己的电脑上获取ip地址（获取自己电脑这一步现在没有用，以后会有用）：

```text
ipconfig
```

如果你是无线的wifi，去找无线局域网下面的IPv4地址；如果是有线网络，去找以太网适配器下面的IPv4地址。

在树莓派上启用ssh:

```text
sudo systemctl enable ssh
sudo systemctl start ssh
```

可以通过下面的代码检查ssh是否已经启用：

```
sudo systemctl status ssh
```

然后在windows命令行里，输入：

```
ssh ubuntu@ip
```

其中，ubuntu是你的树莓派的用户名，ip是树莓派的ip地址。

如果这个时候你的电脑命令行的前缀变成了ubuntu@ubuntu，那么恭喜你，你已经成功连接了ssh, 这个时候你在这个windows窗口里输入命令，和你在树莓派里面输入命令是一样的。

### 3.4 在vscode里面远程连接ssh

你可以在windows的cmd窗口里用ssh直接控制树莓派，但是你还可以用vscode连接上ssh。

为什么要用vscode而不是命令行呢？因为vscode连接上树莓派以后，你可以直接查看树莓派里面的所有文件，而在命令行里，你只能通过vim或者nano查看（这两个东西很麻烦，不熟悉的话很容易自己一不小心改了文件还改不回来，甚至有可能退不出文件编辑界面）

好的，现在打开你的vscode，安装一个叫Remote-SSH的插件。然后Ctrl+Shift+P打开命令面板，输入并选择Remote-SSH: Connect to Host, 选择添加新的SSH主机，然后输入连接命令：

```text
ssh ubuntu@ip
```

这个ubuntu和ip和刚才是一样的，然后输入密码，在vscode的终端里就可以执行和刚才在命令行里一样的操作，同时在vscode左侧的文件列表里可以直接查看和编辑树莓派的所有文件。

### 3.5 给树莓派配置图形界面

#### 3.5.1 配置图形界面

在树莓派里输入以下代码：

```text
sudo apt update
sudo apt upgrade
sudo apt install ubuntu-desktop
```

你可以在最后一条命令后面加上一个-y，这样你可以看到安装的所有细节。

**注意：** 安装ubuntu-desktop需要几个G的网络，如果你使用的是手机热点，请慎重考虑。

安装好以后，在树莓派里（最好不要是ssh里）运行：

```text
sudo reboot
```

你的树莓派会重启，重启完以后和树莓派直接相连的显示屏上就会出现你熟悉的图形界面。

如果你是在ssh里面运行的reboot，会有概率失败。如果成功的话，树莓派会重启，同时你自己电脑上的ssh会退出；如果失败的话，那就什么都不会发生。

**另外，有一点要额外提一下：**

ssh不支持图形界面的转发，也就是说，你不能在除了与树莓派直接连接的显示器以外的任何通过ssh连接的命令行里看到例如gedit和rviz这种的图形界面。如果你一定要在这些地方看图形界面，可以试试下面的远程桌面连接。

#### 3.5.2 远程桌面连接

配置好图形界面以后，树莓派已经可以正常显示桌面了。如果你不想让树莓派接着一个显示屏，或者以后因为树莓派装在来回动小车上而觉得拿着显示屏到处跑非常抽象的话，你可以尝试用远程桌面连接。

**注意：** 远程桌面连接会经常性的出现bug或者连接失败，就算连接上了，显示的桌面分辨率以及处理能力也堪忧，我个人不建议用这种方法。如果真的觉得拿着显示屏跑很抽象，你可以试试X11转发，但是这部分我还没做过，可能以后会做。

在树莓派里，先打开远程桌面功能。具体在Setting - Sharing - Remote desktop, 然后密码什么的自己设置一下。

常见的远程桌面连接有两种方式：

- Windows官方的远程桌面连接：直接在开始菜单搜索远程桌面连接，然后输入树莓派的ip和你刚才设置的密码，就可以连接了。会出现蓝屏和黑屏，卡在某种蓝屏和黑屏的界面都是常见的bug，没有解决方法，直接放弃远程桌面连接就可以。
- 用RealVNC Server: 这种方法确实是ubuntu和树莓派官方推荐的，但是如果上面那个方法都做不到，这个方法基本就废了。但是还是说说细节吧：

在树莓派里运行：

```text
sudo apt install realvnc-vnc-server realvnc-vnc-viewer
```

如果报错说没有这个包，就要自己去realvnc官网上下载ubuntu对应的.deb安装包（[链接](https://www.realvnc.com/en/connect/download/vnc/)），然后通过以下命令安装：

```text
sudo dpkg -i xxx.deb
sudo apt -f install
```

其中，xxx.deb是下载下来的.deb包的名称。然后启用vnc server:

```text
sudo systemctl enable vncserver-x11-serviced.service
sudo systemctl start vncserver-x11-serviced.service
```

如果要求你设置密码的话，就设置一个。

在你自己的电脑上，去[realvnc官网](https://www.realvnc.com/en/connect/download/viewer/)下载并且安装上RealVNC Viewer，运行RealVNC Viewer以后输入你的树莓派ip，再输入你刚才设的密码。如果成功，你会看到ubuntu的桌面。

另外，RealVNC虽然可以是免费的，但是它默认是收费的，需要你通过一个很隐蔽的地方切换到免费版。具体可以看这里：[链接](https://topstip.com/realvnc-is-taking-down-its-free-family-plan-on-17-june/?via=E07513)

### 3.6 设置系统语言

打开系统设置，选择Region & Language, 然后选择Manage Installed Languages.

进入之后选择Install/Remove Languages, 找到Chinese(Simplified)，开始安装。

安装完成以后，在语言列表里把中文拖到最上面，这个时候系统会询问你是否更新系统目录名称，也就是说，你是否要把Downloads这个文件夹改名叫下载，是否要把Videos这个文件夹改名叫视频等等。我的建议是不要改，因为有些应用他只认英文的路径。

以上是一个正常的ubuntu切换成简体中文的过程，如果你的ubuntu像我一样不太正常，设置里找不到这些选项，也可以在终端里操作（终端按Ctrl+Alt+T打开）：

```text
sudo apt update
sudo apt install language-pack-zh-hans language-pack-zh-hans-base
sudo update-locale LANG=zh_CN.UTF-8
sudo locale-gen zh_CN.UTF-8
sudo reboot
```

这样的话就成功安装了中文语言包，系统应该会问你是否更新系统目录名称，这个和刚才是完全一样的，建议不要更新。

## 4. ROS搭建

### 4.1 版本选择

首先你需要清楚，不同版本的ubuntu对应着不同版本的ROS系统：

```text
ubuntu 16 -> ROS kinetic
ubuntu 18 -> ROS melodic
ubuntu 20 -> ROS noetic
ubuntu 22 -> ROS humble(ROS2)
```

humble是ROS2，而其他的是ROS1，也就是说，noetic是ROS1的最后一个版本，也是现在最热门的一个版本。曾经热门的是16对应的kinetic.

那为什么不用humble甚至更新的版本，一定要用ROS1呢？因为ROS2的技术社区还没有完全搭建好，很多的技术还是一片空白，而ROS1已经比较成熟，另外ROS1和ROS2的数据很多是不互通的，突然把所有文件从ROS1变成ROS2也不太容易。

我们安装的是ubuntu 20.04.5 server，所以安装ROS noetic就好。

### 4.2 选择软件源

ROS是一个国外的网站，在国内直接访问的话有的时候会连接不上。但好在国内有很多高校和企业都有ros的镜像，可以调用他们的镜像，这样就避开了需要连国外网站的问题。

先说人在国外的情况，这种情况比较简单（当然如果你某种特殊流量充裕且网速也不错也可以这样做）：

需要先通过代码给定一个软件源，配上这个软件源的密钥。这样当你安装ROS的时候，就会自动定向到这个软件源，如果密钥通过，ROS就会开始安装。

添加ROS官方软件源：

```text
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

再安装一下cURL（这个大小写真的没写错）：

```text
sudo apt install curl
```

curl是一个传输数据的命令行工具，这样你可以直接把ros官网的密钥直接copy的本地：

```text
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

（如果没有权限记得在代码最前面加上sudo）

好的，现在软件源配置好了，跳到下面安装的步骤就可以了。

如果你在国内：

可以看这个教程：[ubuntu安装](https://blog.csdn.net/qq_64671439/article/details/135287166)，下面的东西和这篇文章里面说的差不多。

#### 4.2.1 配置下载服务器

这个和镜像源不太一样，镜像源是你安装ros调用的仓库以及安装好了ros以后更新ros里面所有的东西的时候调用的仓库，而下载服务器是ubuntu系统最底层的设置，影响的是ubuntu的仓库，和ros无关。

但是改成国内的下载服务器下载会快一点，你也可以选择忽略配置下载服务器这一步，影响不大。

具体步骤是：系统设置->软件和更新->下载自，在这里面选国内的镜像源（比如mirrors.aliyun.com），然后关闭就可以了。

#### 4.2.2 配置镜像源

国内有很多的镜像源都可以选，这里列举几个最常用的：

清华源：

```text
sudo sed -i 's#http://packages.ros.org/ros/ubuntu#http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu#g' /etc/apt/sources.list.d/ros-latest.list
```

阿里云源：

```text
sudo sed -i 's#http://packages.ros.org/ros/ubuntu#http://mirrors.aliyun.com/ros/ubuntu#g' /etc/apt/sources.list.d/ros-latest.list
```

中科大源：

```text
sudo sed -i 's#http://packages.ros.org/ros/ubuntu#http://mirrors.ustc.edu.cn/ros/ubuntu#g' /etc/apt/sources.list.d/ros-latest.list
```

选一个你喜欢的镜像源就行，选好以后运行下面的命令看看软件源是不是换过来了：

```text
cat /etc/apt/sources.list.d/ros-latest.list
```

然后再更新一下系统文件(sudo apt update)就可以了。

#### 4.2.3 添加密钥

和上面说的一样，先安装cURL，然后直接去ros官网获取密钥：

```text
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

然后验证密钥是否添加成功：

```text
apt-key list | grep "ROS"
```

当然这一步也可以不做，添加密钥的时候就会直接验证密钥是否有效。但如果说添加了密钥显示的是密钥有问题，你就需要自己去网上找密钥了。前面提到的文章里有一个，但是谁也不能保证这个密钥现在还好用：

```text
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

如果不行就只能拜托你自己去网上搜刮点密钥出来了。或者如果你能访问ROS官方的github页面的话，你可以直接去这里：[链接](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key)

把页面内容保存为ros.key，然后复制到系统目录：

```text
sudo cp ros.key /usr/share/keyrings/ros-archive-keyring.gpg
```

这样密钥就设置好了。现在有两个问题：

- 不同镜像源需要的密钥是不同的吗？

  不是。虽然镜像源的网址不一样，但他们都是ros官方镜像的复制粘贴版本，需要的密钥都是相同的。

- 这个密钥是不可以分享给他人的吗？

  不是。不要被密钥这个名字骗了，密钥只是怕你误安装ros的保护手段，密钥在ros官方那里也都是全公开的。

### 4.3 安装ROS

准备就绪以后就可以安装ROS了，注意这一步需要花掉几个G的网络数据，如果你是用手机热点连接的树莓派的话，请为你的流量考虑一下。

```text
sudo apt update
sudo apt install ros-noetic-desktop-full
```

安装ros的过程会非常漫长，可能会有一个小时甚至两个小时，你可以去干点别的。正常情况下，如果前面十几行代码正常出现没有报错，已经能看到开始安装东西了，那么整个过程就不会报错，放心去做别的事情就行。

### 4.4 安装rosdep

rosdep是ROS的一个库，是用来管理自动安装ros包的系统依赖项的，当你安装一些别的东西需要调用系统级的库的时候，rosdep会自己去找这些库的位置。

或者你可以理解为，当你安装一个ros的库而ros告诉你找不到文件或目录的时候，你就可以调用rosdep了，它会帮你找。

安装过程是这样的：

```text
sudo apt install python3-rosdep2
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update
```

但是基本上，国内的用户安装rosdep都会遇到网络错误，并且国内网站上的很多方法也都没有用，但国内有一个叫“鱼C小甲鱼”的博主做了一个rosdepc，可以完美绕过网络问题而实现rosdep的功能：[链接](https://zhuanlan.zhihu.com/p/398754989)

具体步骤是这样的：

考虑到我们安装的是一个新的ubuntu系统，没有python环境，需要先装一个python环境，同时安装pip：

（注意，python2在2020年就停止维护了，现在用的都是python3，并且python3的技术已经很成熟，不用担心安全和技术问题）

```text
sudo apt update
sudo apt install python3-pip
```

安装rosdepc：

```text
sudo pip3 install rosdepc
sudo rosdepc init
rosdepc update
```

这样就安装好了rosdepc，它可以实现和rosdep一样的功能。但是，其实如果你不安装rosdep，影响也不会那么大，只不过是后面如果安装或者更新什么东西有报错（找不到仓库或目录）的时候需要单独去解决而已，问题不大。

### 4.5 设置环境变量

现在ros已经大致安装好了，但是由于没有配置环境变量，在你每次打开终端像想要运行ros相关功能的时候都需要设置一次环境变量，让终端能够识别到ros的存在。为了解决这个问题，我们可以在.bashrc里面添加ros的bash文件的地址：

```text
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

bash是一种命令行解释器，当你在终端里执行ls（查看当前文件夹下的内容）或者cd（跳转到一个指定的文件夹）的时候就会调用bash.

.bashrc的完整路径是~/.bashrc, ~代表的是主界面，以.开头的文件是隐藏文件，可以在你ubuntu的可视化界面中按下ctrl+H来显示所有隐藏的文件。~/.bashrc是bash的配置文件，它决定了系统默认会调用哪些bash. 比如刚才我们把noetic的bash文件加了进去，那么在你每次启动终端的时候，都会调用一次noetic/setup.bash，也就是调用ros环境。

另外，假如说你要修改~/.bashrc（或者说ubuntu系统里面的任意一个文件）的内容，就需要编辑器，常见的有三种：

- gedit: 一个可视化的编辑器，运行gedit以后就会跳出一个像word一样的界面，你可以直接在里面编辑，编辑好以后保存退出就可以。

```text
sudo apt install gedit
sudo gedit ~/.bashrc
```

gedit后面接文件的路径就可以打开了。

- nano：一个只能在命令行里面显示文件内容的编辑器，有很多的快捷指令可以对文档进行操作。

但是强烈建议你在nano里面只用上下左右的箭头控制光标，因为动鼠标滚轮感觉上很不舒服。

安装过程是这样的：

```text
sudo apt install nano
sudo nano ~/.bashrc
```

如果你要显示行号：

```text
sudo nano -l ~/.bashrc
```

nano默认编辑的时候是不会自动缩进的，如果你需要自动缩进：

```text
sudo nano -i ~/.bashrc
```

如果你需要跳转到指定的行，在打开文件以后按ctrl+_，然后输入行号就可以；如果你需要粘贴文本，需要按ctrl+u，而不是ctrl+v.

另外，编辑好文件以后想保存然后退出，不是直接把终端关了，而是按ctrl+x, 他会问你是否保存，选择y保存就可以退出了。

- vim: vim是vi的增强版，你可以认为vim和vi是同一个编辑器，但是vim也是在命令行里编辑的编辑器，没有可视化的界面。

其实vim的功能非常强大，只是我们用不到那么强大的功能而已。

下面是安装步骤：

```text
sudo apt install vim
sudo vim ~/.bashrc
```

进入vim以后，你处于的是普通模式，这个模式是只读的，你不能对文件进行修改。如果你需要修改的话，按e进入插入模式（不用担心按了e以后文件里会不会多一个e，在普通模式里你按什么都不会把按的东西输入进去），就可以用箭头的上下左右控制光标，然后编辑文件了。

编辑好以后，按esc回到普通模式。

在普通模式里，有很多的快捷键，这里举几个常用的例子：

  - dd: 删除当前行
  - yy: 复制当前行
  - p: 粘贴
  - gg: 跳转到文件的第一行
  - u: 撤销（也就是平时用的ctrl+z)

在普通模式里，如果你按下冒号键，就会进入命令模式，命令模式的常用操作如下（下面的冒号都是你刚刚按完的那个，不用再输入一次冒号）：

  - :w: 保存文件
  - :q：退出vim
  - :wq: 保存并退出vim
  - :q!: 不保存并退出vim
  - :set number: 显示行号

### 4.6 其他安装

首先需要安装rosinstall以及它的配套功能。你可以理解为rosinstall-generator生成一个应用清单，然后rosinstall按照清单来安装应用，同时下好应用以后，wstool负责日常管理这些应用。平时的应用升级、应用更新什么的都是wstool来进行的。安装过程如下：

```text
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool
```

还需要安装roslaunch. 它是用来启动launch文件的，launch文件是用来直接启动多个rosrun指令的，这样你就不需要一个一个rosrun来启动了。

```text
sudo apt install python3-roslaunch
```

### 4.7 运行roscore

正如它的名字一样，roscore是ros的核心，只有你的终端运行了roscore，你才能使用ros环境，你只需要在命令行里面输入:

```text
roscore
```

就可以启动roscore了，想执行其他命令，你需要单独开一个新的终端去执行。如果想停止roscore服务，直接按ctrl+c就可以。

如果启动过程中有报错，大概率是一开始安装ros的时候有东西遗漏了（因为最开始装ros只会装一部分的包，刚才又装了rosdep, roslaunch还有rosinstall，系统发现和这些包绑定的一些包没有安），你只需要再运行一次安装ros的指令：

```text
sudo apt install ros-noetic-desktop-full
```

放心，这次安装的东西比较少，会很快就安装完，安装好以后就可以正常启动roscore了。

### 4.8 用海龟仿真器验证ros是否成功启动

如果你已经成功运行了roscore，打开一个新终端，输入：

```text
sudo apt install ros-noetic-turtlesim
rosrun turtlesim turtlesim_node
```

你会看到一个海龟飘在一个蓝色背景上，再打开一个新的终端，输入：

```text
rosrun turtlesim turtle_teleop_key
```

你可以通过箭头的上下左右键来控制海龟移动了，如果海龟成功移动，证明你的ros没有任何问题，按q就可以退出键盘操作了。如果你想关闭海龟仿真器，就在海龟仿真器的窗口按ctrl+c.


