package org.frc5687.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;

public class SimDriveIO implements DriveIO {
    private final Pigeon2 _imu;
    private final Pigeon2SimState _imuSim;
    
    private final double loopPeriodSecs = 0.02;  // 50Hz Make constant i
    private Rotation2d prevYaw = new Rotation2d();

    public SimDriveIO(int pigeonId) {
        _imu = new Pigeon2(pigeonId);
        _imuSim = _imu.getSimState();
        _imuSim.setSupplyVoltage(12.0);
    }

    @Override
    public void updateInputs(DriveInputs inputs) {
        _imuSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        inputs.yawPosition = Rotation2d.fromDegrees(_imu.getYaw().getValueAsDouble());
        inputs.pitchPosition = Rotation2d.fromDegrees(_imu.getPitch().getValueAsDouble());
        inputs.rollPosition = Rotation2d.fromDegrees(_imu.getRoll().getValueAsDouble());

        inputs.yawVelocityRadPerSec = inputs.yawPosition.minus(prevYaw).getRadians() / loopPeriodSecs;
        inputs.pitchVelocityRadPerSec = 0.0; 
        inputs.rollVelocityRadPerSec = 0.0; 

        prevYaw = inputs.yawPosition;
    }

    @Override
    public void writeOutputs(DriveOutputs outputs) {
        // No outputs to write for drive IO
    }

    @Override
    public void reset() {
        _imuSim.setRawYaw(0);
        _imuSim.setPitch(0);
        _imuSim.setRoll(0);
        prevYaw = new Rotation2d();
    }
}