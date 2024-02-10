import time
import brickpi3

BP = brickpi3.BrickPi3()

def read_light_sensor():
    BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_LIGHT_ON)
    while True:
        # read and display the sensor value
        # BP.get_sensor retrieves a sensor value.
        # BP.PORT_1 specifies that we are looking for the value of sensor port 1.
        # BP.get_sensor returns the sensor value (what we want to display).
        try:
            value = BP.get_sensor(BP.PORT_1)
            print(value)                         # print the value
        except brickpi3.SensorError as error:
            print(error)
        
        time.sleep(0.02)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.


def main():
    
    print(BP.get_voltage_battery(),"V\n")
    try:
        read_light_sensor()

    # except the program gets interrupted by Ctrl+C on the keyboard.
    except KeyboardInterrupt: 
        BP.reset_all() 

if __name__ == "__main__":
    main()