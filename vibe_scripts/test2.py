import pygame
import hid
import time

# Initialize Pygame
pygame.init()

# Set up the display
screen = pygame.display.set_mode((640, 480))
pygame.display.set_caption('Joy-Con Rumble Test')

# Vendor ID and Product IDs for Joy-Con
VENDOR_ID = 0x057e  # Nintendo
PRODUCT_ID_LEFT = 0x2006  # Joy-Con Left
# PRODUCT_ID_RIGHT = 0x2007  # Joy-Con Right

def find_joycon(product_id):
    for device in hid.enumerate():
        if device['vendor_id'] == VENDOR_ID and device['product_id'] == product_id:
            return device['path']
    return None

joycon_left_path = find_joycon(PRODUCT_ID_LEFT)
# joycon_right_path = find_joycon(PRODUCT_ID_RIGHT)

if not joycon_left_path: #or not joycon_right_path:
    print("Joy-Con not found!")
    exit()

joycon_left = hid.device()
# joycon_right = hid.device()

joycon_left.open_path(joycon_left_path)
# joycon_right.open_path(joycon_right_path)

# Enable vibration for both Joy-Cons
joycon_left.write([0x01, 0x48, 0x01])
# joycon_right.write([0x01, 0x48, 0x01])

def send_rumble(joycon, duration=0.2):
    # Rumble data
    rumble_data = [
        0x10, 0x80, 0x00, 0x01,
        0x40, 0x40, 0x00, 0x01,
        0x40, 0x40, 0x00, 0x01,
        0x40, 0x40, 0x00, 0x01,
    ]
    joycon.write(rumble_data)
    time.sleep(duration)
    joycon.write([0x10, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                send_rumble(joycon_left)
                # send_rumble(joycon_right)

    screen.fill((0, 0, 0))
    pygame.display.flip()

joycon_left.close()
# joycon_right.close()
pygame.quit()
