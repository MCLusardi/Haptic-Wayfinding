import pygame
from bluepy.btle import Peripheral, DefaultDelegate
import time

# Initialize Pygame
pygame.init()

# Set up the display
screen = pygame.display.set_mode((640, 480))
pygame.display.set_caption('Joy-Con Rumble Test')

# Joy-Con MAC addresses (replace with your actual addresses)
LEFT_JOYCON_MAC = '60:1A:C7:E3:39:5E'
# RIGHT_JOYCON_MAC = 'YY:YY:YY:YY:YY:YY'

class JoyConDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

def connect_joycon(mac_address):
    print(f"Connecting to {mac_address}...")
    joycon = Peripheral(mac_address)
    joycon.setDelegate(JoyConDelegate())
    return joycon

try:
    left_joycon = connect_joycon(LEFT_JOYCON_MAC)
    # right_joycon = connect_joycon(RIGHT_JOYCON_MAC)
except Exception as e:
    print(f"Error connecting to Joy-Con: {e}")
    exit()

def send_rumble(joycon, duration=0.2):
    # Construct the HID report for rumble
    rumble_data = bytearray([
        0x10, 0x80, 0x00, 0x01,
        0x40, 0x40, 0x00, 0x01,
        0x40, 0x40, 0x00, 0x01,
        0x40, 0x40, 0x00, 0x01,
    ])
    # Write the rumble data to the correct handle
    joycon.writeCharacteristic(0x10, rumble_data)
    time.sleep(duration)
    # Stop the rumble
    stop_rumble_data = bytearray([0x10, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    joycon.writeCharacteristic(0x10, stop_rumble_data)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                send_rumble(left_joycon)
                # send_rumble(right_joycon)

    screen.fill((0, 0, 0))
    pygame.display.flip()

left_joycon.disconnect()
# right_joycon.disconnect()
pygame.quit()
