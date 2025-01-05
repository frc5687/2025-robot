package org.frc5687.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;

public class DriveIOCTRE implements DriveIO {
    private final SwerveModule[] _modules;
    private final Pigeon2 _imu;
    private double _yawOffset;

    public DriveIOCTRE(Pigeon2 imu) {
        _imu = imu;
        _modules = new SwerveModule[4];
        
        _modules[0] = new SwerveModule(
                Constants.DriveTrain.NORTH_WEST_CONFIG,
                RobotMap.CAN.TALONFX.NORTH_WEST_ROTATION,
                RobotMap.CAN.TALONFX.NORTH_WEST_TRANSLATION,
                RobotMap.CAN.CANCODER.ENCODER_NW);

        _modules[1] = new SwerveModule(
                Constants.DriveTrain.NORTH_EAST_CONFIG,
                RobotMap.CAN.TALONFX.NORTH_EAST_ROTATION,
                RobotMap.CAN.TALONFX.NORTH_EAST_TRANSLATION,
                RobotMap.CAN.CANCODER.ENCODER_NE);

        _modules[2] = new SwerveModule(
                Constants.DriveTrain.SOUTH_WEST_CONFIG,
                RobotMap.CAN.TALONFX.SOUTH_WEST_ROTATION,
                RobotMap.CAN.TALONFX.SOUTH_WEST_TRANSLATION,
                RobotMap.CAN.CANCODER.ENCODER_SW);

        _modules[3] = new SwerveModule(
                Constants.DriveTrain.SOUTH_EAST_CONFIG,
                RobotMap.CAN.TALONFX.SOUTH_EAST_ROTATION,
                RobotMap.CAN.TALONFX.SOUTH_EAST_TRANSLATION,
                RobotMap.CAN.CANCODER.ENCODER_SE);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        // This will probably work but can be cleaned to be isolated to a function that is part of module
        for (int i = 0; i < 4; i++) {
            _modules[i].periodic();
            inputs.measuredStates[i] = _modules[i].getState();
            inputs.measuredPositions[i] = _modules[i].getPosition();
        }

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
        }
    }

    @Override
    public void zeroGyroscope() {
        _yawOffset = _imu.getYaw().getValue().in(Units.Degrees);
    }
}