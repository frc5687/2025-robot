package org.frc5687.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.subsystems.drive.modules.CTRESwerveModuleIO;
import org.frc5687.robot.subsystems.drive.modules.SwerveModule;
import org.frc5687.robot.subsystems.drive.modules.SwerveModuleIO;

public class CTREDriveIO implements DriveIO {
    private final Pigeon2 _imu;
    private final StatusSignal<Angle> _yaw;
    private final StatusSignal<Angle> _pitch;
    private final StatusSignal<Angle> _roll;
    private final StatusSignal<AngularVelocity> _yawVelocity;
    private final StatusSignal<AngularVelocity> _pitchVelocity;
    private final StatusSignal<AngularVelocity> _rollVelocity;

    private final SwerveModule[] _modules;

    public CTREDriveIO(int pigeonId, String canbus) {
        _imu = new Pigeon2(pigeonId, canbus);
        _modules = new SwerveModule[Constants.DriveTrain.NUM_MODULES];

        // Get signals
        _yaw = _imu.getYaw();
        _pitch = _imu.getPitch();
        _roll = _imu.getRoll();
        _yawVelocity = _imu.getAngularVelocityZDevice();
        _pitchVelocity = _imu.getAngularVelocityYDevice();
        _rollVelocity = _imu.getAngularVelocityXDevice();

        SwerveModuleIO northWestIO =
                new CTRESwerveModuleIO(
                        Constants.DriveTrain.NW_CONFIG,
                        RobotMap.CAN.TALONFX.NORTH_WEST_TRANSLATION,
                        RobotMap.CAN.TALONFX.NORTH_WEST_ROTATION,
                        RobotMap.CAN.CANCODER.ENCODER_NW,
                        Constants.DriveTrain.CAN_BUS);

        SwerveModuleIO northEastIO =
                new CTRESwerveModuleIO(
                        Constants.DriveTrain.NE_CONFIG,
                        RobotMap.CAN.TALONFX.NORTH_EAST_TRANSLATION,
                        RobotMap.CAN.TALONFX.NORTH_EAST_ROTATION,
                        RobotMap.CAN.CANCODER.ENCODER_NE,
                        Constants.DriveTrain.CAN_BUS);

        SwerveModuleIO southWestIO =
                new CTRESwerveModuleIO(
                        Constants.DriveTrain.SW_CONFIG,
                        RobotMap.CAN.TALONFX.SOUTH_WEST_TRANSLATION,
                        RobotMap.CAN.TALONFX.SOUTH_WEST_ROTATION,
                        RobotMap.CAN.CANCODER.ENCODER_SW,
                        Constants.DriveTrain.CAN_BUS);

        SwerveModuleIO southEastIO =
                new CTRESwerveModuleIO(
                        Constants.DriveTrain.SE_CONFIG,
                        RobotMap.CAN.TALONFX.SOUTH_EAST_TRANSLATION,
                        RobotMap.CAN.TALONFX.SOUTH_EAST_ROTATION,
                        RobotMap.CAN.CANCODER.ENCODER_SE,
                        Constants.DriveTrain.CAN_BUS);
        _modules[0] = new SwerveModule(Constants.DriveTrain.NW_CONFIG, northWestIO);
        _modules[1] = new SwerveModule(Constants.DriveTrain.NE_CONFIG, northEastIO);
        _modules[2] = new SwerveModule(Constants.DriveTrain.SW_CONFIG, southWestIO);
        _modules[3] = new SwerveModule(Constants.DriveTrain.SE_CONFIG, southEastIO);

        // Set up synchronized signals
        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0, _yaw, _pitch, _roll, _yawVelocity, _pitchVelocity, _rollVelocity);
    }

    @Override
    public void updateInputs(DriveInputs inputs) {
        BaseStatusSignal.refreshAll(_yaw, _pitch, _roll, _yawVelocity, _pitchVelocity, _rollVelocity);

        // Little worried how this might work, can we enforce that modules will ALWAYS update its
        // measurements before the drivetrain?
        for (int i = 0; i < Constants.DriveTrain.NUM_MODULES; i++) {
            inputs.measuredStates[i] = _modules[i].getState();
            inputs.modulePositions[i] = _modules[i].getPosition();
        }

        inputs.yawPosition = Rotation2d.fromDegrees(_yaw.getValueAsDouble());
        inputs.pitchPosition = Rotation2d.fromDegrees(_pitch.getValueAsDouble());
        inputs.rollPosition = Rotation2d.fromDegrees(_roll.getValueAsDouble());

        inputs.yawVelocityRadPerSec = Units.degreesToRadians(_yawVelocity.getValueAsDouble());
        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(_pitchVelocity.getValueAsDouble());
        inputs.rollVelocityRadPerSec = Units.degreesToRadians(_rollVelocity.getValueAsDouble());
    }

    @Override
    public void writeOutputs(DriveOutputs outputs) {
        for (int i = 0; i < Constants.DriveTrain.NUM_MODULES; i++) {
            _modules[i].setDesiredState(outputs.desiredStates[i]);
        }
    }

    @Override
    public void reset() {
        _imu.setYaw(0);
    }

    @Override
    public void runCharacterization(double output) {
        for (int i = 0; i < Constants.DriveTrain.NUM_MODULES; i++) {
            _modules[i].runCharacterization(output);
        }
    }
}
