# EMCU diagnostics message
std_msgs/Header header

# Value from Roboclaw's Read Status command
uint32 status

float64 battery_voltage
float64 logic_voltage
float64 temperature

# Amperage
float64 motor1_current
float64 motor2_current
