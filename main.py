

import json
import paho.mqtt.client as mqtt
import time
import gpiozero
from gpiozero import LED, Button, OutputDevice
import board
import adafruit_dht
import requests


RASPBERRY_ID = 4  # ID of the site


LOCAL_BROKER = "0.0.0.0"
LOCAL_BROKER_PORT = 1883

REMOTE_BROKER = "test.mosquitto.org"
REMOTE_BROKER_PORT = 1883

TOPIC_REGISTER = "/plant/register"
TOPIC_PUBLISH_UPDATES = "/UA/iotProj/plants/"
TOPIC_PUBLISH_SITE = "/UA/iotProj/sites/"
TOPIC_RECEIVE_UPDATES = "/UA/iotProj/plants/edit"
TOPIC_ESP_FEEDBACK = "/plant/register/response"
TOPIC_MOISTURE_TEMPLATE = "/plant/{id}/moisture"

KEY_LAST_WATERED = "last_watered"

# LED_PIN = 27
PIN_WATER_SENSOR = 22   #
PIN_RELAY = 23
PIN_DHT = board.D17

# led = LED(LED_PIN)
water_sensor = Button(PIN_WATER_SENSOR)
relay = OutputDevice(PIN_RELAY, active_high=True, initial_value=False)

TESTING_FACTOR = 0.2  # Factor to speed up the testing process
# watering parameters
MAX_MOISTURE_READINGS = 5   # Number of moisture readings to store for each plant and calculate avg from
WATER_COOLDOWN = 300 * TESTING_FACTOR   # Time in seconds to wait before watering the same plant again
MOISTURE_OUTDATED_THRESHOLD = 200 * TESTING_FACTOR # Time in seconds after which the moisture reading is considered outdated. Cannot be smaller than esp loop interval
WATER_PUMP_BURST_TIME = 5 # Time in seconds to run the water pump
MIN_TEMP_THRESHOLD = 10  # Minimum temperature for watering


MAIN_ITERATION_PERIOD = 60 * TESTING_FACTOR  # Time in seconds to wait between main loop iterations


plant_dict = {}  # dictionary to store plant data: moisture, last_watered, target_moisture, last_update
#  shape: {"plant_id": {"moisture": [], "last_watered": 0, "target_moisture": 0, "last_update": date}}

subscribed_plant_topics = []  # list of topics to which the script is subscribed

site_state = {"id": RASPBERRY_ID, "temperature": 25, "humidity": None, "rain": False, "water_level": 1, "last_update": time.time()}


def on_subscribe(client, userdata, mid, reason_code_list, properties):
    # Since we subscribed only for a single channel, reason_code_list contains
    # a single entry
    if reason_code_list[0].is_failure:
        print(f"Broker rejected you subscription: {reason_code_list[0]}")
    else:
        print(f"Broker granted the following QoS: {reason_code_list[0].value}")


def on_message(client: mqtt.Client, userdata, mes):
    print("message received", mes.payload.decode("utf-8"), "on", mes.topic, mes.mid)
    msg = mes.payload.decode("utf-8")

    # TODO: restructure the mqtt logic, since esp32 can send json, we can have a fixed topic for all moisture
    #  readings, no need to register

    if mes.topic == TOPIC_REGISTER:
        print("received register request with id:", msg)
        try:
            plant_id = int(msg)
            plant_topic = TOPIC_MOISTURE_TEMPLATE.format(id=plant_id)
            if plant_id not in plant_dict.keys():
                plant_dict[plant_id] = {"moisture": [], KEY_LAST_WATERED: None,
                                        "target_moisture": 60, "last_update": None}

                print(f"Registered new plant with ID: {plant_id}")

                # create a topic for the plant
            # notify the esp32 with the topic to publish on
                # subscribe to the topic
                client.subscribe(plant_topic, 2)
                subscribed_plant_topics.append(plant_topic)
            client.publish(TOPIC_ESP_FEEDBACK, plant_topic, 2)

        except ValueError:
            print("Invalid plant ID")

    # if already registered, update the moisture reading
    elif mes.topic in subscribed_plant_topics:
        try:
            data = json.loads(msg)
            p_id = int(data["id"])
            moisture = float(data["moisture"])
            plant_dict[p_id]["moisture"].append(moisture)
            plant_dict[p_id]["last_update"] = time.time()
            if len(plant_dict[p_id]["moisture"]) > MAX_MOISTURE_READINGS:
                plant_dict[p_id]["moisture"] = plant_dict[p_id]["moisture"][1:]
            print(f"Received moisture reading {moisture} for plant {p_id}")
        except ValueError:
            print("Invalid plant ID")

    else:
        print("received message on topic", mes.topic)
        print("message payload:", mes.payload.decode("utf-8"))
        print("message qos:", mes.qos)
        print("message retain flag:", mes.retain)


def on_message_remote(client: mqtt.Client, userdata, mes):
    print("message received", mes.payload.decode("utf-8"), "on", mes.topic, mes.mid)
    msg = mes.payload.decode("utf-8")
    if mes.topic == TOPIC_RECEIVE_UPDATES:
        try:
            data = json.loads(msg)
            p_id = int(data["id"])
            target_moisture = int(data["target_moisture"])
            plant_dict[p_id]["target_moisture"] = target_moisture
            print(f"Updated target moisture for plant {p_id} to {target_moisture}")
        except ValueError:
            print("Invalid plant ID")


def on_publish(client, userdata, mid, reason_code, properties=None):
    print("published message id:", mid)


def on_connect(client, userdata, flags, rc, properties=None):
    print("connected with result code", rc)
    # subscribe to the register topic


def water_plant(plant_id):

    relay.on()
    time.sleep(WATER_PUMP_BURST_TIME)
    relay.off()
    plant_dict[plant_id][KEY_LAST_WATERED] = time.time()


def check_rain():
    # TODO: get weather data from API
    return False


def on_water_sensor_high():
    print("Water sensor high")
    site_state["water_level"] = 0
    # TODO


def on_water_sensor_low():
    print("Water sensor low")
    site_state["water_level"] = 1
    # TODO


def main_loop():
    while True:

        # ----------------------- Read Sensors -----------------------
        try:
            site_state["temperature"] = sensor_dht.temperature
            site_state["humidity"] = sensor_dht.humidity
        except RuntimeError as error:
            # Errors happen fairly often, DHT's are hard to read, just keep going
            print(error.args[0])

        # ----------------------- decide watering -----------------------
        if site_state["temperature"] is not None and site_state["temperature"] > MIN_TEMP_THRESHOLD:
            will_rain_today = check_rain()
            if not will_rain_today:

                for plant_id, data in plant_dict.items():

                    # if the plant has not been watered in the last WATER_COOLDOWN seconds and the moisture reading
                    # is not outdated
                    if data["last_update"] is not None and time.time() - data["last_update"] < MOISTURE_OUTDATED_THRESHOLD \
                            and (data[KEY_LAST_WATERED] is None or time.time() - data[KEY_LAST_WATERED] > WATER_COOLDOWN):

                        # if the average moisture is below the target, water the plant
                        avg_moisture = sum(data["moisture"]) / len(data["moisture"])
                        if avg_moisture < data["target_moisture"]:

                            if site_state["water_level"] == 1:
                                print(f"Plant {plant_id} needs watering. Activating pump...")
                                water_plant(plant_id)
                                data[KEY_LAST_WATERED] = time.time()
                            else:
                                print("Water level too low. Cannot water plants. Refill")

        # ----------------------- Publish Site State -----------------------
        client2.publish(TOPIC_PUBLISH_UPDATES, json.dumps(site_state), 2)
        client2.publish(TOPIC_PUBLISH_UPDATES, json.dumps(plant_dict), 2)

        # Mqtt with API stopped working, so we are using https requests to send the data

        json_list = []
        for plant_id, plant_data in plant_dict.items():
            # Extract moisture value
            if plant_data["moisture"]:
                moist = plant_data["moisture"][-1] # Get the last moisture reading
            else:
                moist = 0.0
            # moisture = plant_data["moisture"]

            # Create a new dictionary with desired properties
            plant_json = {
                "moisture": moist,
                "light": 0,  # Set light to 0 (api expects data)
                "plantId": plant_id
            }

            json_list.append(plant_json)

        print(requests.post("https://iotproject1api20240501175507.azurewebsites.net/api/Plants/stats",
                      json=json_list,
                      headers={"Content-Type": "application/json"}, ))
        # print(requests.post("https://iotproject1api20240501175507.azurewebsites.net/api/PlantSites/stats",
        #               json=json.dumps(site_state),
        #               headers={"Content-Type": "application/json"}, ))


        print("site state:", site_state)

        time.sleep(MAIN_ITERATION_PERIOD)


# Start of main script
# water level callbacks
site_state["water_level"] = water_sensor.value*-1+1
water_sensor.when_activated = on_water_sensor_high
water_sensor.when_deactivated = on_water_sensor_low
water_sensor.when_held = on_water_sensor_high

sensor_dht = adafruit_dht.DHT11(PIN_DHT)


# init mqtt clients
client1 = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

client1.on_message = on_message
client1.on_publish = on_publish
client1.on_connect = on_connect

client1.connect(LOCAL_BROKER, LOCAL_BROKER_PORT)
client1.loop_start()
client1.subscribe(TOPIC_REGISTER, 2)

client2 = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

client2.on_message = on_message_remote
client2.on_publish = on_publish
client2.on_connect = on_connect

client2.connect(REMOTE_BROKER, REMOTE_BROKER_PORT)
client2.loop_start()
client2.subscribe(TOPIC_RECEIVE_UPDATES, 2)

try:
    main_loop()
except KeyboardInterrupt:
    print("Script stopped by user")
    client1.disconnect()
    client2.disconnect()
    relay.off()
    exit()
