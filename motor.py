from machine import Timer,PWM
from fpioa_manager import fm
from Maix import GPIO
from board import board_info

class Motor():
    def __init__(self,inverse=[False, False]):
        if isinstance(inverse, list) and len(inverse) == 2:
            pass
        else:
            raise 'inverse must be a list of size 2, like [False, False]'
        self.inverse = inverse

        # init gpio that adjust motors' direction
        fm.register(board_info.LDIR1,fm.fpioa.GPIO0)
        fm.register(board_info.LDIR2,fm.fpioa.GPIO1)
        fm.register(board_info.RDIR1,fm.fpioa.GPIO2)
        fm.register(board_info.RDIR2,fm.fpioa.GPIO3)
        self.ldir1 = GPIO(GPIO.GPIO0,GPIO.OUT)
        self.ldir2 = GPIO(GPIO.GPIO1,GPIO.OUT)
        self.rdir1 = GPIO(GPIO.GPIO2,GPIO.OUT)
        self.rdir2 = GPIO(GPIO.GPIO3,GPIO.OUT)
        self.ldir1.value(0)
        self.ldir2.value(0)
        self.rdir1.value(0)
        self.rdir2.value(0)

        # init pwm channel
        tim1 = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
        self.ch1 = PWM(tim1, freq=500000, duty=50, pin=board_info.PWM_L)

        tim2 = Timer(Timer.TIMER0, Timer.CHANNEL1, mode=Timer.MODE_PWM)
        self.ch2 = PWM(tim2, freq=500000, duty=50, pin=board_info.PWM_R)

    def run(self,left_speed, right_speed):
        if self.inverse[0] == True:
            left_speed = -left_speed
        if self.inverse[1] == True:
            right_speed = -right_speed

        if left_speed < 0:
            self.ldir1.value(0)
            self.ldir2.value(1)
            left_speed = -left_speed
        else:
            self.ldir1.value(1)
            self.ldir2.value(0)
        if left_speed > 100:
            left_speed = 100
        self.ch1.duty(int(left_speed))

        if right_speed < 0:
            self.rdir1.value(0)
            self.rdir2.value(1)
            right_speed = -right_speed
        else:
            self.rdir1.value(1)
            self.rdir2.value(0)
        if right_speed > 100:
            right_speed = 100
        self.ch2.duty(int(right_speed))

    def stop(self):
        return self.run(0,0)
