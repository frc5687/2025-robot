package org.frc5687.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.units.Units;

public class DriveIOSim implements DriveIO {
    private final SwerveModule[] _modules;
    private final Pigeon2 _imu;
    private final Pigeon2SimState _imuSim;
    private double _yawOffset;

    public DriveIOSim() {
        _imu = new Pigeon2(RobotMap.CAN.PIGEON.PIGEON);
        _imuSim = _imu.getSimState();
        
        _modules = new SwerveModule[4];
        
        // I dont really like this, maybe follow 6328 and add to constructor
        _modules[0] = new SwerveModule(
                Constants.DriveTrain.NORTH_WEST_CONFIG,
                RobotMap.CAN.TALONFX.NORTH_WEST_ROTATION,
                RobotMap.CAN.TALONFX.NORTH_WEST_TRANSLATION,
                RobotMap.CAN.CANCODER.ENCODER_NW) {
            @Override
            protected ModuleIO createIO(int driveID, int turnID, int encoderID, String canBus, double offset) {
                return new ModuleIOSim();
            }
        };

        _modules[1] = new SwerveModule(
                Constants.DriveTrain.NORTH_EAST_CONFIG,
                RobotMap.CAN.TALONFX.NORTH_EAST_ROTATION,
                RobotMap.CAN.TALONFX.NORTH_EAST_TRANSLATION,
                RobotMap.CAN.CANCODER.ENCODER_NE) {
            @Override
            protected ModuleIO createIO(int driveID, int turnID, int encoderID, String canBus, double offset) {
                return new ModuleIOSim();
            }
        };

        _modules[2] = new SwerveModule(
                Constants.DriveTrain.SOUTH_WEST_CONFIG,
                RobotMap.CAN.TALONFX.SOUTH_WEST_ROTATION,
                RobotMap.CAN.TALONFX.SOUTH_WEST_TRANSLATION,
                RobotMap.CAN.CANCODER.ENCODER_SW) {
            @Override
            protected ModuleIO createIO(int driveID, int turnID, int encoderID, String canBus, double offset) {
                return new ModuleIOSim();
            }
        };

        _modules[3] = new SwerveModule(
                Constants.DriveTrain.SOUTH_EAST_CONFIG,
                RobotMap.CAN.TALONFX.SOUTH_EAST_ROTATION,
                RobotMap.CAN.TALONFX.SOUTH_EAST_TRANSLATION,
                RobotMap.CAN.CANCODER.ENCODER_SE) {
            @Override
            protected ModuleIO createIO(int driveID, int turnID, int encoderID, String canBus, double offset) {
                return new ModuleIOSim();
            }
        };
        
        if (Utils.isSimulation()) {
            _imu.getYaw().setUpdateFrequency(100);
            _imu.getAngularVelocityZDevice().setUpdateFrequency(100);
        }
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        //Logger.recordOutput("Drive Inputs", inputs);
        _imuSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        for (int i = 0; i < 4; i++) {
            _modules[i].periodic();
            inputs.measuredStates[i] = _modules[i].getState();
            inputs.measuredPositions[i] = _modules[i].getPosition();
        }

        var chassisSpeeds = Constants.DriveTrain.KINEMATICS.toChassisSpeeds(inputs.measuredStates);
        _imuSim.addYaw(Units.Rotations.convertFrom(chassisSpeeds.omegaRadiansPerSecond * Constants.UPDATE_PERIOD, Units.Radians));

        double yawDegrees = BaseStatusSignal.getLatencyCompensatedValue(
            _imu.getYaw(),
            _imu.getAngularVelocityZDevice()
        ).in(Units.Degrees);
        inputs.heading = Rotation2d.fromDegrees(yawDegrees - _yawOffset);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < _modules.length; i++) {
            _modules[i].setGoalState(states[i]);
            _modules[i].updateDashboard(); // Hack in TODO: Make updates for subsystem automatic, potentailly subsystem amanger again
        }
    }

    @Override
    public Rotation2d getHeading() {
        return _imu.getRotation2d();
    }

    @Override
    public void zeroGyroscope() {
        _yawOffset = _imu.getYaw().getValue().in(Units.Degrees);
    }
}