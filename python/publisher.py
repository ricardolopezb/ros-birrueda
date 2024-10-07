import paho.mqtt.client as mqtt

# MQTT broker details
broker = 'localhost'  # Replace with your MQTT broker address
port = 1883
topic = "cli/commands"

client = mqtt.Client()
client.connect(broker, port)
# Function to publish a command
def publish_command(command):
    client.publish(topic, command)
    print(f"Published command: {command}")

if __name__ == "__main__":
    command = input("Enter the CLI command to publish: ")
    publish_command(command)
    client.disconnect()
