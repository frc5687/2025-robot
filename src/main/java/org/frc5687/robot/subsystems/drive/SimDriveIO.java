package org.frc5687.robot.subsystems.drive;

import org.frc5687.robot.Constants;
import org.frc5687.robot.subsystems.drive.modules.SimSwerveModuleIO;
import org.frc5687.robot.subsystems.drive.modules.SwerveModule;
import org.frc5687.robot.subsystems.drive.modules.SwerveModuleIO;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class SimDriveIO implements DriveIO {
    private final Pigeon2 _imu;
    private final Pigeon2SimState _imuSim;
    
    private final SwerveDriveKinematics _kinematics;
    private final double loopPeriodSecs = 0.02;  // 50Hz Make constant i
    private Rotation2d prevYaw = new Rotation2d();

    private SwerveModule[] _modules;

    public SimDriveIO(int pigeonId) {
        _imu = new Pigeon2(pigeonId);
        _imuSim = _imu.getSimState();
        _imuSim.setSupplyVoltage(12.0);
        _kinematics = new SwerveDriveKinematics(Constants.DriveTrain.MODULE_LOCATIONS);
        _modules = new SwerveModule[Constants.SwerveModule.NUM_MODULES];

        SwerveModuleIO northWestIO = new SimSwerveModuleIO(
                Constants.DriveTrain.NW_CONFIG);

        SwerveModuleIO northEastIO = new SimSwerveModuleIO(
                Constants.DriveTrain.NE_CONFIG);

        SwerveModuleIO southWestIO = new SimSwerveModuleIO(
                Constants.DriveTrain.SW_CONFIG);

        SwerveModuleIO southEastIO = new SimSwerveModuleIO(
                Constants.DriveTrain.SE_CONFIG);

        // This is ok. We should make this enforced as a part of DriveIO
        _modules[0] = new SwerveModule(Constants.DriveTrain.NW_CONFIG, northWestIO);
        _modules[1] = new SwerveModule(Constants.DriveTrain.NE_CONFIG, northEastIO);
        _modules[2] = new SwerveModule(Constants.DriveTrain.SW_CONFIG, southWestIO);
        _modules[3] = new SwerveModule(Constants.DriveTrain.SE_CONFIG, southEastIO);
    }

    @Override
    public void updateInputs(DriveInputs inputs) {
        _imuSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        for (int i = 0; i < Constants.DriveTrain.NUM_MODULES; i++) {
            inputs.measuredStates[i] = _modules[i].getState();
            inputs.modulePositions[i] = _modules[i].getPosition();
        }

        ChassisSpeeds speeds =_kinematics.toChassisSpeeds(inputs.measuredStates);
        _imuSim.addYaw(Units.radiansToDegrees(speeds.omegaRadiansPerSecond * loopPeriodSecs));

        inputs.yawPosition = Rotation2d.fromDegrees(_imu.getYaw().getValueAsDouble());
        inputs.pitchPosition = Rotation2d.fromDegrees(_imu.getPitch().getValueAsDouble());
        inputs.rollPosition = Rotation2d.fromDegrees(_imu.getRoll().getValueAsDouble());

        inputs.yawVelocityRadPerSec = inputs.yawPosition.minus(prevYaw).getRadians() / loopPeriodSecs;
        inputs.pitchVelocityRadPerSec = 0.0; 
        inputs.rollVelocityRadPerSec = 0.0; 

        prevYaw = inputs.yawPosition;
        // System.out.println("SimDriveIO Update time: " + Timer.getTimestamp());
    }

    @Override
    public void writeOutputs(DriveOutputs outputs) {
        for (int i = 0; i < Constants.DriveTrain.NUM_MODULES; i++) {
            _modules[i].setDesiredState(outputs.desiredStates[i]);
        }
    }

    @Override
    public void reset() {
        _imuSim.setRawYaw(0);
        _imuSim.setPitch(0);
        _imuSim.setRoll(0);
        prevYaw = new Rotation2d();
    }

    @Override
    public void runCharacterization(double output) {
        for (int i = 0; i < Constants.DriveTrain.NUM_MODULES; i++) {
            _modules[i].runCharacterization(output);
        }
    }
}