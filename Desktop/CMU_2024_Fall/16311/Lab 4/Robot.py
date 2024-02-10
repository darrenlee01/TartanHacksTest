import time
import brickpi3
import math

class Robot:
    def __init__(self, BP):
        self.BP = BP
        self.angVelR = 0
        self.angVelL = 0
        self.prevError = 0
        self.error = 0
    
    def voltage_okay(self):
        if(self.BP.get_voltage_battery() > 9):
            return True
        print("Voltage is too low!")
        return False
    
    def cmd_motor_pows(self, lpow, rpow):
        print('commanding ', lpow, ' ', rpow)
        self.BP.set_motor_power(self.BP.PORT_A, 1 * lpow)  # flipping directionality of motors
        self.BP.set_motor_power(self.BP.PORT_B, 1 * rpow)  # flipping directionality of motors
    
    def stop(self):
        self.BP.reset_all()

    def read_light_sensor(self, port):
        # self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.NXT_LIGHT_ON)
        self.BP.set_sensor_type(port, self.BP.SENSOR_TYPE.NXT_LIGHT_ON)

        # read and display the sensor value
        # BP.get_sensor retrieves a sensor value.
        # BP.PORT_1 specifies that we are looking for the value of sensor port 1.
        # BP.get_sensor returns the sensor value (what we want to display).
        try:
            value = self.BP.get_sensor(port)
            print(value)     
            return value                    # print the value
        except:
            return -1
                # except brickpi3.SensorError as error:
                #     print(error)
                #     return -1
                # time.sleep(0.04)

    def get_light_sensor_error(self, port1, port2)
        leftLight = self.read_light_sensor(port1)
        rightLight = self.read_light_sensor(port2)
        error = rightLight - leftLight 
        self.prevError = self.error
        self.error = error
    
    def PD_controller_power(self, port1, port2, kp , kd, dt):
        self.get_light_sensor_error(port1,port2)
        p = kp * self.error
        d = kd * (self.error - self.prevError)/dt
        power = p + d
        return power


    def getangVelWheel(self, t):
        self.BP.offset_motor_encoder(self.BP.PORT_B,self.BP.get_motor_encoder(self.BP.PORT_B))
        initDegR = self.BP.get_motor_encoder(self.BP.PORT_B)
        self.BP.offset_motor_encoder(self.BP.PORT_A,self.BP.get_motor_encoder(self.BP.PORT_A))
        initDegL = self.BP.get_motor_encoder(self.BP.PORT_A)
        time.sleep(t)
        newDegR = self.BP.get_motor_encoder(self.BP.PORT_B)
        newDegL = self.BP.get_motor_encoder(self.BP.PORT_A)
        self.angVelL = ((newDegL - initDegL)/t) * math.pi/180
        self.angVelR = ((newDegR - initDegR)/t) * math.pi/180
        # print("angVelR: ", self.angVelR)
        # print("angVelL: ", self.angVelL)
        return

    def getangVelL(self, t):
        self.BP.offset_motor_encoder(self.BP.PORT_A,self.BP.get_motor_encoder(self.BP.PORT_A))
        initDeg = self.BP.get_motor_encoder(self.BP.PORT_A)
        time.sleep(t)
        newDeg = self.BP.get_motor_encoder(self.BP.PORT_A)
        self.angVelL = ((newDeg - initDeg)/t) * math.pi/180
        # print("angVelL: ", self.angVelL)
        return 

    def angVelRobot(self,t, wheelbase, diameter):
        tanVelR = self.angVelR * diameter/2
        tanVelL = self.angVelL * diameter/2
        #wheelbase = 4.84
        #diameter = 2.1875
        # print("angVelRobot: ", (tanVelR - tanVelL)/wheelbase)
        return (tanVelR - tanVelL)/wheelbase
    
    def tanVelRobot(self, diameter):
        tanVelR = self.angVelR * diameter/2 
        tanVelL = self.angVelL * diameter/2 
        # print("tanVelRobot: ", (tanVelL+ tanVelR)/2)
        return (tanVelL+ tanVelR)/2

    def rangeKutta(self, params, tanVel, angVel, t):
        x = params[0]
        y = params[1]
        theta = params[2]
        k00 = tanVel * math.cos(theta)
        k01 = tanVel * math.sin(theta)
        k02 = angVel
        k10 = tanVel * math.cos(theta + t/2 * k02)
        k11 = tanVel * math.sin(theta + t/2 * k02)
        k12 = angVel
        k20 = tanVel * math.cos(theta + t/2 * k12)
        k21 = tanVel * math.sin(theta + t/2 * k12)
        k22 = angVel
        k30 = tanVel * math.cos(theta + t * k22)
        k31 = tanVel * math.sin(theta + t * k22)
        k32 = angVel
        
        xNew = x + t/6 * (k00 + 2 * (k10 + k20) + k30)
        yNew = y + t/6 * (k01 + 2 * (k11 + k21) + k31)
        thetaNew = theta + t/6 * (k02 + 2 * (k12 + k22) + k32)

        return (xNew, yNew, thetaNew)

