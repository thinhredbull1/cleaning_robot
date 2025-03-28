import json

import paho.mqtt.client as mqtt
import threading
import time


from datetime import datetime

broker = "broker.emqx.io"  # Broker
port = 1883

topic_subscribe = "gps_rbotss/device1_recv"
topic_publish = "gps_rbotss/device1_send"

lat = 21.03627055
lon = 105.7869495
bat = 76
speed = 0

def on_message(client, userdata, message):
    #print(f"Received message: {message.payload.decode()} on topic {message.topic}")
    data_rcv = json.loads(message.payload.decode())
    new_lat = data_rcv["lat"]
    new_lon = data_rcv["lon"]
    state = data_rcv["state"]
    print(f"New loc: {new_lat}, {new_lon}")
    print(f"State: {state}")


def mqtt_subscriber():
    client = mqtt.Client()
    client.on_message = on_message
    client.connect(broker, port)
    client.subscribe(topic_subscribe)
    client.loop_forever()

def mqtt_publisher():
    client = mqtt.Client()
    client.connect(broker, port)

    while True:
        datetime_send = datetime.now()
        message = {
            "lat": lat,
            "lon": lon,
            "speed": speed,
            "time": datetime_send.strftime("%Y-%m-%d %H:%M:%S"),
            "bat": bat
        }
        data_send = json.dumps(message)
        client.publish(topic_publish, data_send)
        print(f"Sent: {data_send}")
        time.sleep(5)


thread_subscriber = threading.Thread(target=mqtt_subscriber)
thread_publisher = threading.Thread(target=mqtt_publisher)

thread_subscriber.start()
thread_publisher.start()
