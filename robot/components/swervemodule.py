import math
import wpilib
import ctre
from networktables import NetworkTables
from wpimath.controller import PIDController
from collections import namedtuple

ModuleConfig = namedtuple('ModuleConfig', ['sd_prefix', 'zero', 'inverted', 'allow_reverse'])

ENCODER_SIZE = 4096

class SwerveModule:

    def __init__(self, config, drive, rotate):

        self.driveMotor = drive
        self.rotateMotor = rotate

        self.cfg = config

        self.encoder = self.cfg["encoder"]

        self.encoder_zero = self.cfg["zero"]

        self.inverted = self.cfg["inverted"]
        self.allow_reverse = self.cfg["allow_reverse"]

        self.driveMotor.setInverted(self.inverted)

        self.requested_speed = 0

        self.pid_controller = PIDController(0.0005, 0.0, 0.0)
        self.pid_controller.enableContinuousInput(0.0, 4096.0)
        self.pid_controller.setTolerance(5, 5)
        
    def get_encoder_ticks(self):
        return self.encoder.getSelectedSensorPosition()

    def flush(self):
        self.pid_controller.setSetpoint(self.encoder.getSelectedSensorPosition()) #self.encoder.getSelectedSensorPosition()
        self.requested_speed = 0
        self.pid_controller.reset()
    
    @staticmethod
    def ticks_to_degrees(ticks):
        isNegative = ticks < 0
        deg = (abs(ticks) % ENCODER_SIZE)/ENCODER_SIZE
        if isNegative: deg *= -1
        deg *= 360
        return deg

    @staticmethod
    def degree_to_ticks(degree):
        return (degree / 360) * ENCODER_SIZE
    
    def set_deg(self, value):
        self.pid_controller.setSetpoint((self.degree_to_ticks(value) + self.encoder_zero) % ENCODER_SIZE)

    def move(self, speed, deg):
        deg %= 360

        if self.allow_reverse:

            if abs(deg - self.ticks_to_degrees(self.get_encoder_ticks())) > 90:
                speed *= -1
                deg += 180
                deg %= 360
            
        self.requested_speed = speed
        self.set_deg(deg)
        
    def execute(self):
        error = self.pid_controller.calculate(self.encoder.getSelectedSensorPosition())
        output = 0

        if not self.pid_controller.atSetpoint():
            output = max(min(error, 1), -1)

        self.rotateMotor.set(output)
        self.driveMotor.set(max(min(self.requested_speed, 0.7), -0.7)) 
        # ORIGINAL VALUE AT 0.5 SUBJECT TO CHANGE
            

        
        