import Robot

BP = brickpi3.BrickPi3()
def balanceRobot():
    Robot = Robot(BP)
    port1 = Robot.BP.PORT_1
    port2 = Robot.BP.PORT_2
    kp = 0
    kd = 0
    dt = 0
    if(not Robot.voltage_okay):
        print("Robot voltage is too low")
        return -1
    while(True):
        power = Robot.PD_controller_power(port1, port2, kp, kd, dt)
        Robot.cmd_motor_pows(power,power)
