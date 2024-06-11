from rumble_test import *

if __name__ == "__main__":
    joycon_id_right = get_R_id()
    joyconR = RumbleJoyCon(*joycon_id_right)

    amp = 0.5 #Range 0 - 1
    freq = 320 #Range 41 Hz - 1253 Hz
    t = 0.5

    while t>0:
        for i in range(15):
            if amp == 0.5: amp = 0
            elif amp == 0: amp = 0.5 

            print("t:", t)
            data = RumbleData(freq/2, freq, amp)

            b = data.GetData()
            # print(b)

            joyconR._send_rumble(b)
            if amp == 0.5: time.sleep(0.1)
            else: time.sleep(t)

        t -= 0.1