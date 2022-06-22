# This message is for command from rr_pi_control to rr_command_collector
# gear, speed and brack is send to control speed

Header header

# Gear definition
uint8 kGearForward=0
uint8 kGearNeutral=1
uint8 kGearBackward=2

# Gear shift. 0: forward, 1: neutral, 2: backward, 3-255: neutral
uint8 gear

uint16 speed   #0~1000
uint8 brake     #brake (1~100)

