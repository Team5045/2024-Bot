import wpilib
import ctre
from magicbot import MagicRobot
from collections import namedtuple
from networktables import NetworkTables, NetworkTable
from networktables.util import ntproperty

from components import swervedrive, swervemodule

ModuleConfig = swervemodule.ModuleConfig

# Download and install stuff on the RoboRIO after imaging
'''
py -3 -m robotpy_installer download-python
py -3 -m robotpy_installer install-python
py -3 -m robotpy_installer download robotpy
py -3 -m robotpy_installer install robotpy
py -3 -m robotpy_installer download robotpy[ctre]
py -3 -m robotpy_installer install robotpy[ctre]
py -3 -m robotpy_installer download robotpy[rev]
py -3 -m robotpy_installer install robotpy[rev]
py -3 -m robotpy_installer download pynetworktables
py -3 -m robotpy_installer install pynetworktables
py -3 -m pip install -U robotpy[ctre]
py -3 -m pip install robotpy[ctre]
'''

# Push code to RoboRIO (only after imaging)
'''
python robot/robot.py deploy --skip-tests
py robot/robot.py deploy --skip-tests --no-version-check
'''


class MyRobot(MagicRobot):


    def createObjects(self):

        NetworkTables.initialize(server='roborio-5045-frc.local')
        self.sd: NetworkTable = NetworkTables.getTable('SmartDashboard')
        self.controller = wpilib.XboxController(0)

        self.frontLeftModule_driveMotor = ctre.WPI_TalonSRX(5) # 1, 2 og
        self.frontLeftModule_rotateMotor = ctre.WPI_TalonSRX(8)

        self.frontRightModule_driveMotor = ctre.WPI_TalonSRX(7) # 3, 4 og
        self.frontRightModule_rotateMotor = ctre.WPI_TalonSRX(6)
        
        self.rearRightModule_driveMotor = ctre.WPI_TalonSRX(1) # 5, 6 og
        self.rearRightModule_rotateMotor = ctre.WPI_TalonSRX(2)

        self.rearLeftModule_driveMotor = ctre.WPI_TalonSRX(3) # 7, 8 og
        self.rearLeftModule_rotateMotor = ctre.WPI_TalonSRX(4)

        self.frontLeftModule_encoder = self.frontLeftModule_rotateMotor
        self.frontRightModule_encoder = self.frontRightModule_rotateMotor
        self.rearLeftModule_encoder = self.rearLeftModule_rotateMotor
        self.rearRightModule_encoder = self.rearRightModule_rotateMotor

        self.frontLeftModule_cfg = {"sd_prefix":'frontLeft_Module', "zero": -101.0, "inverted":True, "allow_reverse":True, "encoder":self.frontLeftModule_encoder}
        self.frontRightModule_cfg = {"sd_prefix":'frontRight_Module', "zero": 0, "inverted":False, "allow_reverse":True, "encoder":self.frontRightModule_encoder}
        self.rearLeftModule_cfg = {"sd_prefix":'rearLeft_Module', "zero": 0, "inverted":True, "allow_reverse":True, "encoder":self.rearLeftModule_encoder}
        self.rearRightModule_cfg = {"sd_prefix":'rearRight_Module', "zero": 0, "inverted":False, "allow_reverse":True, "encoder":self.rearRightModule_encoder}

        self.frontLeftModule = swervemodule.SwerveModule(self.frontLeftModule_cfg, self.frontLeftModule_driveMotor, self.frontLeftModule_rotateMotor)
        self.frontRightModule = swervemodule.SwerveModule(self.frontRightModule_cfg, self.frontRightModule_driveMotor, self.frontRightModule_rotateMotor)
        self.rearLeftModule = swervemodule.SwerveModule(self.rearLeftModule_cfg, self.rearLeftModule_driveMotor, self.rearLeftModule_rotateMotor)
        self.rearRightModule = swervemodule.SwerveModule(self.rearRightModule_cfg, self.rearRightModule_driveMotor, self.rearRightModule_rotateMotor)

        self.drive = swervedrive.SwerveDrive(self.frontLeftModule, self.frontRightModule, self.rearLeftModule, self.rearRightModule)

    def autonomousInit(self):
        # self.drive.flush()
        pass
    
    def teleopInit(self):
        # self.drive.flush()
        pass
    
    def move(self, y, x, rcw):
        self.drive.move(y, x, rcw)

    def teleopPeriodic(self):
        print(" ")
        self.move(self.controller.getLeftY(), self.controller.getLeftX(), self.controller.getRightX())
        self.drive.execute()

        # Encoder Positions % 4096
        self.sd.putValue("FL_encoder_pos", self.frontLeftModule_encoder.getSelectedSensorPosition() % 4096)
        self.sd.putValue("FR_encoder_pos", self.frontRightModule_encoder.getSelectedSensorPosition() % 4096)
        self.sd.putValue("RL_encoder_pos", self.rearLeftModule_encoder.getSelectedSensorPosition() % 4096)
        self.sd.putValue("RR_encoder_pos", self.rearRightModule_encoder.getSelectedSensorPosition() % 4096)
        # Module setpoints
        self.sd.putValue("FL_setpoint", self.frontLeftModule.pid_controller.getSetpoint())
        self.sd.putValue("FR_setpoint", self.frontRightModule.pid_controller.getSetpoint())
        self.sd.putValue("RL_setpoint", self.rearLeftModule.pid_controller.getSetpoint())
        self.sd.putValue("RR_setpoint", self.rearRightModule_encoder.pid_controller.getSetpoint())
        

if __name__ == "__main__":
    wpilib.run(MyRobot)