import paho.mqtt.client as mqtt
import subprocess

# MQTT broker details
broker = '190.221.174.11'  # Replace with your MQTT broker address
port = 1883
topic = "cli/commands"

# Callback when a message is received
def on_message(client, userdata, message):
    command = message.payload.decode("utf-8")
    print(f"Received command: {command}")
    try:
        # Execute the received CLI command
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        print(f"Output:\n{result.stdout}")
        if result.stderr:
            print(f"Error:\n{result.stderr}")
    except Exception as e:
        print(f"Failed to execute command: {e}")

# Set up the MQTT client
client = mqtt.Client()

client.on_message = on_message
client.connect(broker, port)

# Subscribe to the topic and listen
client.subscribe(topic)
print(f"Subscribed to topic: {topic}")

client.loop_forever()  # Loop to keep the subscriber running
