import time
import brickpi3

BP = brickpi3.BrickPi3()
voltage = BP.get_voltage_battery()
print(voltage, "V")

BP.reset_all()