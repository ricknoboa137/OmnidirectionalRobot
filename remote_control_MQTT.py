from paho.mqtt import client as mqtt_client
import random 
import pygame
import time
import json

# Define the IP address and port for broadcasting
broker = '192.168.0.110'#'192.168.0.108'  # Broadcast to all devices on the network
port = 1883
topic = "gamepad/joystick"
cont = 1

# Generate a Client ID with the publish prefix.
client_id = f'ImageProvider-{random.randint(0, 1000)}'
# username = 'emqx'
# password = 'public'

################################################################################
def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1,client_id)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client
##################################################################################

def publish(client, msg):
    msg_count = 1
    #print(msg)
    result = client.publish(topic, msg) # result: [0, 1]
    status = result[0]
    if status == 0:
        msg_count += 1
        #print("Message Printed")
    else:
        print(f"Failed to send message to topic {topic}")
    
    
##################################################################################


client = connect_mqtt()
client.loop_start()
# Initialize Pygame
pygame.init()

# Check if there's a joystick connected
if not pygame.joystick.get_count():
    print("Error: No joystick detected.")
    quit()

# Get the first joystick (modify the index for other connected controllers)
joystick = pygame.joystick.Joystick(0)

# Get the name of the joystick
joystick_name = joystick.get_name()
print(f"Connected joystick: {joystick_name}")
running = True
clock = pygame.time.Clock()  # Used for recording timerunning = True
clock.tick(60)  # Update at most 60 times per second


while cont == 1:
    #cont =2
    #publish(client, "Helloooo")
    # Check if there's a joystick connected
    #pygame.init()
    joystick_data = []
    if not pygame.joystick.get_count():
        print("Error: No joystick detected.")
        pygame.quit()
        client.loop_stop()
        #quit()
    pygame.event.get()
    axes = joystick.get_numaxes()
    buttons = joystick.get_numbuttons()

    joystick_info = json.dumps({
        "joystick_id": 1,
        "name": joystick.get_name(),
        "axes": [joystick.get_axis(j) for j in range(axes)],
        "buttons": [joystick.get_button(j) for j in range(buttons)],
        "num_axes": axes,
        "num_buttons": buttons
    })
    #joystick_data.append(joystick_info)

    x_axis = joystick.get_axis(0)  # Left thumbstick horizontal (-1 to 1)
    y_axis = joystick.get_axis(1)  # Left thumbstick vertical (-1 to 1)
    publish(client, joystick_info)
    time.sleep(0.1)  # Adjust sleep time as needed


client.loop_stop()
# Quit Pygame
pygame.quit()
#cap.release()
##################################################################################
