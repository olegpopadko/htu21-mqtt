import paho.mqtt.client as mqtt
import json
import smbus
import time

def read_humidity():
    bus = smbus.SMBus(config['i2c_bus_number'])

    # SI7021 address, 0x40(64)
    # 0xF5(245) Select Relative Humidity HOLD master mode
    bus.write_byte(0x40, 0xE5)

    time.sleep(0.3)

    # SI7021 address, 0x40(64)
    # Read data back, 2 bytes, Humidity MSB first
    data0 = bus.read_byte(0x40)
    data1 = bus.read_byte(0x40)

    return ((data0 * 256 + data1) * 125 / 65536.0) - 6

def read_temperature():
    bus = smbus.SMBus(config['i2c_bus_number'])

    # SI7021 address, 0x40(64)
    # 0xF3(243) Select temperature HOLD master mode
    bus.write_byte(0x40, 0xE3)

    time.sleep(0.3)

    # SI7021 address, 0x40(64)
    # Read data back, 2 bytes, Temperature MSB first
    data0 = bus.read_byte(0x40)
    data1 = bus.read_byte(0x40)

    # Convert the data
    return ((data0 * 256 + data1) * 175.72 / 65536.0) - 46.85

with open('config.json') as file_data:
    config = json.load(file_data)

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(config['humidity_refresh_topic'])
    client.subscribe(config['temperature_refresh_topic'])

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    if msg.topic == config['humidity_refresh_topic']:
        current_humidity = read_humidity()
        client.publish(config['humidity_topic'], current_humidity)
    if msg.topic == config['temperature_refresh_topic']:
        current_temperature = read_temperature()
        client.publish(config['temperature_topic'], current_temperature)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(config['mqtt_host'], 1883, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()
