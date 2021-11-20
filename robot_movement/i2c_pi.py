from smbus import SMBus
import time

addr = 0x8
wheel_speed_cmd = 10
bus = SMBus(1)
time.sleep(5)

num = 1
while num == 1:
    input_send = int(input(">>>>     "))

    if input_send < 0:
        sign_bit_R, sign_bit_L = 1, 1
        input_send = input_send * -1
    else:
        sign_bit_R, sign_bit_L = 0, 0

    binary = bin(input_send)
    binary_len = len(binary)
    if binary_len <= 10:
        print(wheel_speed_cmd, 4, [sign_bit_L, sign_bit_R, input_send, input_send])
        bus.write_block_data(addr, wheel_speed_cmd, [sign_bit_L, sign_bit_R, input_send, input_send])
    else:
        byte_1 = int(binary[2:binary_len-8], 2)
        byte_2 = int(binary[binary_len-8:binary_len], 2)
        print(wheel_speed_cmd, 2, [sign_bit_L, byte_1, byte_2, sign_bit_R, byte_1, byte_2])
        bus.write_block_data(addr, wheel_speed_cmd, [sign_bit_L, sign_bit_R, byte_1, byte_2, byte_1, byte_2])
        # bus.write_word_data(addr, byte_1, byte_2)
