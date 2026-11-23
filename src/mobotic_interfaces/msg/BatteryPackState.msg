# Individual pack telemetry. Status and health codes follow sensor_msgs/BatteryState.
uint8 POWER_SUPPLY_STATUS_UNKNOWN=0
uint8 POWER_SUPPLY_STATUS_CHARGING=1
uint8 POWER_SUPPLY_STATUS_DISCHARGING=2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING=3
uint8 POWER_SUPPLY_HEALTH_GOOD=1
uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE=5
uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE=7
uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN=0

std_msgs/Header header
float32 voltage            # Volts
float32 current            # Amperes, negative when discharging
float32 percentage         # State of charge as a fraction from 0 to 1
uint8 power_supply_status
uint8 power_supply_health
uint8 power_supply_technology
bool present
float32[] cell_voltage
string location
string serial_number
