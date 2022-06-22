# this is Extended ERP42 Feedback message
# Some information with converted units are included

Header header

# speed in m/s. this is SI unit
float64 speed_mps

# speed in km/h. this is commonly used.
float64 speed_kph

# steer in radian. negative is left. this is SI unit.
float64 steer_rad

# steer in degree. negative is left. this is commonly used.
float64 steer_deg

# brake in ratio. mapping [1 , 100] to [0.0, 1.0]
float64 brake_ratio
