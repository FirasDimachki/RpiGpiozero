import paho.mqtt.client as mqtt
import time
import gpiozero
from gpiozero import LED, Button, OutputDevice



led = LED(27)
button = Button(22)

button.when_pressed = led.on
button.when_released = led.off

cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
cl.connect('127.0.0.1', 1883)
cl.loop_start()

time.sleep(10)

cl.loop_stop()
cl.disconnect()
led.off()

exit()
