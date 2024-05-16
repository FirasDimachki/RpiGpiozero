'''

This was used to test a checklist of packages that were installed on the Raspberry Pi.

Not to be used as part of the project

'''








import paho.mqtt.client as mqtt
import time
import gpiozero
from gpiozero import LED, Button, OutputDevice
import board

import adafruit_dht
LED_PIN = 27
BUTTON_PIN = 22
RELAY_PIN = 23
DHT_PIN = board.D17

# gpiod test
led = LED(LED_PIN)
button = Button(BUTTON_PIN)
relay = OutputDevice(RELAY_PIN, active_high=True, initial_value=False)

button.when_pressed = led.on
button.when_released = led.off


def on_subscribe(client, userdata, mid, reason_code_list, properties):
    # Since we subscribed only for a single channel, reason_code_list contains
    # a single entry
    if reason_code_list[0].is_failure:
        print(f"Broker rejected you subscription: {reason_code_list[0]}")
    else:
        print(f"Broker granted the following QoS: {reason_code_list[0].value}")


def on_unsubscribe(client, userdata, mid, reason_code_list, properties):
    # Be careful, the reason_code_list is only present in MQTTv5.
    # In MQTTv3 it will always be empty
    if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
        print("unsubscribe succeeded (if SUBACK is received in MQTTv3 it success)")
    else:
        print(f"Broker replied with failure: {reason_code_list[0]}")
    client.disconnect()


def on_message(client, userdata, message):
    # userdata is the structure we choose to provide, here it's a list()
    #userdata.append(message.payload)
    # We only want to process 10 messages
    #if len(userdata) >= 10:
    #    client.unsubscribe("/paho/test/topic")
    msg=message.payload.decode("utf-8")
    print("Received ",msg," on topic ", message.topic)

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        # we should always subscribe from on_connect callback to be sure
        # our subscribed is persisted across reconnections.
        client.subscribe("/paho/test/topic", 2)


def on_publish(client, userdata, mid, reason_code, properties):
    # reason_code and properties will only be present in MQTTv5. It's always unset in MQTTv3
    print(f"mid: {mid}")


mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_connect = on_connect
mqttc.on_message = on_message
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe
mqttc.on_publish = on_publish
mqttc.connect('127.0.0.1', 1883)
mqttc.loop_start()

# Our application produce some messages
msg_info = mqttc.publish("/paho/test/topic", "my message", qos=2)


sensor_dht = adafruit_dht.DHT11(DHT_PIN)

try:
    # Print the values to the serial port
    temperature_c = sensor_dht.temperature
    humidity = sensor_dht.humidity
    print("Temp={0:0.1f}ºC, Humidity={1:0.1f}%".format(temperature_c, humidity))
except RuntimeError as error:
    # Errors happen fairly often, DHT's are hard to read, just keep going
    print(error.args[0])
    time.sleep(2.0)
# test dht

time.sleep(10)

mqttc.loop_stop()
mqttc.disconnect()
led.off()

exit()
