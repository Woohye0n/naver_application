# This message represents raw command of erp42
# This message follows given protocol from unmanned solution

# Mode definition
uint8 kModeManual=0
uint8 kModeAuto=1

# Etop status definition
uint8 kEStopOff=0
uint8 kEStopOn=1

# Gear definition
uint8 kGearForward=0
uint8 kGearNeutral=1
uint8 kGearBackward=2

Header header

# Auto or Manual mode. 0 is Manual, 1 is Auto
uint8 a_or_m

# Emergency Stop. 0 is E-Stop off, 1 is E-Stop on
uint8 e_stop

# Gear shift. 0: forward, 1: neutral, 2: backward, 3-255: neutral
uint8 gear

# Motor Raw command. (0-1000)
uint16 speed

# steering degree * 71. negative is left. maximum 4% error.
int16 steer

# brake scale. 1: no braking, 100: full braking
uint8 brake

# Increasing each one step (0-255)
uint8 alive
