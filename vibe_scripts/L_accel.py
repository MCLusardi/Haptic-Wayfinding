from rumble_test import *

if __name__ == "__main__":
    joycon_id_left = get_L_id()
    joyconL = RumbleJoyCon(*joycon_id_left)

    amp = 0.5 #Range 0 - 1
    freq = 320 #Range 41 Hz - 1253 Hz
    t = 0.5

    amp_x_values = []
    amp_values = []
    amp_t = 0

    while t>0:
        for i in range(15):
            if amp == 0.5: amp = 0
            elif amp == 0: amp = 0.5 

            print("t:", t)
            data = RumbleData(freq/2, freq, amp)

            b = data.GetData()
            # print(b)

            amp_x_values.append(amp_t)
            amp_values.append(amp)
            joyconL._send_rumble(b)
            if amp == 0.5: 
                time.sleep(0.1)
                amp_t += 0.1
            else: 
                time.sleep(t)
                amp_t += t
            

        t -= 0.1

    # plt.plot(amp_x_values, amp_values)
    # plt.set_title('Amplitude Values')
    # plt.set_xlabel('time (seconds)')
    # plt.set_ylabel('Amplitude')