from teensy_serial_backend import TeensyComms

SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200
TIMEOUT_MS = 500

teensy = TeensyComms()
teensy.setup(SERIAL_PORT, BAUD_RATE, TIMEOUT_MS)

user_input = input("Enter a string to send to the Teensy:\n")
response = teensy.send_msg(user_input + "\n")
print(f"Teensy responded: {response}")