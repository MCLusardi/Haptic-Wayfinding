from rumble_test import *

if __name__ == "__main__":
    joycon_id_right = get_R_id()
    joyconR = RumbleJoyCon(*joycon_id_right)

    amp = 0.5 #Range 0 - 1
    freq = 500 #Range 41 Hz - 1253 Hz

    while True:
        print("freq:", freq)
        data = RumbleData(freq/2, freq, amp)

        b = data.GetData()
        # print(b)

        joyconR._send_rumble(b)
        time.sleep(.01)

        #increase amp for next round
        freq -= 25
        if freq <50:
            freq = 320