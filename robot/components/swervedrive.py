from magicbot import magiccomponent
from components import swervemodule
import math

class SwerveDrive:

    def __init__(self, FR, FL, BR, BL):
        self.frontRightModule = FR
        self.frontLeftModule = FL
        self.rearRightModule = BR
        self.rearLeftModule = BL

        self.modules = {
            "front_right": self.frontRightModule,
            "front_left": self.frontLeftModule,
            "rear_right": self.rearRightModule,
            "rear_left": self.rearLeftModule
        }

        self.requested_angles = {
            "front_right": 0,
            "front_left": 0,
            "rear_right": 0,
            "rear_left": 0
        }

        self.requested_speeds = {
            "front_right": 0,
            "front_left": 0,
            "rear_right": 0,
            "rear_left": 0
        }

        self.requested_vectors = {
            "fwd": 0,
            "strafe": 0,
            "rcw": 0
        }

        self.width = (28 / 12) / 2 # (Inch / 12 = Foot) / 2
        self.length = (28 / 12) / 2 # (Inch / 12 = Foot) / 2

    def orient(self):
        self.encoder_mod_set = 0.0
        
    def flush(self):
        '''Function will be called to reset motors (make them flush)'''

        self.requested_angles = {
            "front_right": 0,
            "front_left": 0,
            "rear_right": 0,
            "rear_left": 0
        }

        self.requested_speeds = {
            "front_right": 0,
            "front_left": 0,
            "rear_right": 0,
            "rear_left": 0
        }

        self.requested_vectors = {
            "fwd": 0,
            "strafe": 0,
            "rcw": 0
        }
        for module in self.modules.values():
            module.flush()

    def set_raw_fwd(self, fwd):
        self.requested_vectors['fwd'] = fwd
    
    def set_raw_strafe(self, strafe):
        self.requested_vectors['strafe'] = strafe

    def set_raw_rcw(self, rcw):
        self.requested_vectors['rcw'] = rcw
    
    def move(self, fwd, strafe, rcw):
        self.set_raw_fwd(fwd)
        self.set_raw_strafe(strafe)
        self.set_raw_rcw(rcw)
    
    def calculate_vectors(self):
        
        ratio = math.hypot(self.length, self.width)
        
        # Velocity of quadrants
        frontX = self.requested_vectors['strafe'] - (self.requested_vectors['rcw'] * (self.length / ratio))
        rearX = self.requested_vectors['strafe'] + (self.requested_vectors['rcw'] * (self.length / ratio))
        leftY = self.requested_vectors['fwd'] - (self.requested_vectors['rcw'] * (self.width / ratio))
        rightY = self.requested_vectors['fwd'] + (self.requested_vectors['rcw'] * (self.width / ratio))

        # Speed for quadrants
        frontLeft_speed = math.hypot(frontX, rightY)
        frontLeft_angle = 0
        frontLeft_angle = math.degrees(math.atan2(frontX, rightY))
        
        frontRight_speed = math.hypot(frontX, leftY)
        frontRight_angle = 0
        frontRight_angle = math.degrees(math.atan2(frontX, leftY))
      
        rearLeft_speed = math.hypot(rearX, rightY)
        rearLeft_angle = 0
        rearLeft_angle = math.degrees(math.atan2(rearX, rightY))
        
        rearRight_speed = math.hypot(rearX, leftY)
        rearRight_angle = 0
        rearRight_angle = math.degrees(math.atan2(rearX, leftY)) 

        self.requested_speeds['front_left'] = frontLeft_speed
        self.requested_speeds['front_right'] = frontRight_speed
        self.requested_speeds['rear_left'] = rearLeft_speed
        self.requested_speeds['rear_right'] = rearRight_speed

        self.requested_angles['front_left'] = frontLeft_angle
        self.requested_angles['front_right'] = frontRight_angle
        self.requested_angles['rear_left'] = rearLeft_angle
        self.requested_angles['rear_right'] = rearRight_angle

    def execute(self):
        self.calculate_vectors()
        for i in self.modules:
            self.modules[i].move(self.requested_speeds[i], self.requested_angles[i])
        
        for i in self.modules:
            self.modules[i].execute()