from smbus import SMBus
import time

addr = 0x8
wheel_speed_cmd = 10
bus = SMBus(1)
time.sleep(5)

wheel_command = 1000

num = 1
while num == 1:
    time.sleep(0.1)
    input_send = wheel_command
    
    read_data = bus.read_i2c_block_data(addr, 16, 4)
    encoder_speed = [((read_data[0] << 8) + read_data[1]) / 10.0, ((read_data[2] << 8) + read_data[3]) / 10.0]
    print(encoder_speed)

    if input_send < 0:
        sign_bit_R, sign_bit_L = 1, 1
        input_send = input_send * -1
    else:
        sign_bit_R, sign_bit_L = 0, 0

    binary = bin(input_send)
    binary_len = len(binary)
    if binary_len <= 10:
        bus.write_block_data(addr, wheel_speed_cmd, [sign_bit_L, sign_bit_R, input_send, input_send])
    else:
        byte_1 = int(binary[2:binary_len-8], 2)
        byte_2 = int(binary[binary_len-8:binary_len], 2)
        bus.write_block_data(addr, wheel_speed_cmd, [sign_bit_L, sign_bit_R, byte_1, byte_2, byte_1, byte_2])
