from tuning import *
from machine import LED

led = LED("LED_BLUE")
led.on()

thresholds = [
(35, 54, 48, 87, -4, 67),#red
(31, 65, -2, 69, -100, -55)#blue
]

tuning = PanTuning(thresholds, gain = 5, p=0.2, i=0, d=0.005)

tuning.measure(0.1)
