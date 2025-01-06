package org.frc5687.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class CTREDriveIO implements DriveIO {
    private final Pigeon2 _imu;
    private final StatusSignal<Angle> _yaw;
    private final StatusSignal<Angle> _pitch;
    private final StatusSignal<Angle> _roll;
    private final StatusSignal<AngularVelocity> _yawVelocity;
    private final StatusSignal<AngularVelocity> _pitchVelocity;
    private final StatusSignal<AngularVelocity> _rollVelocity;

    public CTREDriveIO(int pigeonId, String canbus) {
        _imu = new Pigeon2(pigeonId, canbus);
        
        // Get signals
        _yaw = _imu.getYaw();
        _pitch = _imu.getPitch();
        _roll = _imu.getRoll();
        _yawVelocity = _imu.getAngularVelocityZDevice();
        _pitchVelocity = _imu.getAngularVelocityYDevice();
        _rollVelocity = _imu.getAngularVelocityXDevice();

        // Set up synchronized signals
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            _yaw, _pitch, _roll,
            _yawVelocity, _pitchVelocity, _rollVelocity
        );
    }

    @Override
    public void updateInputs(DriveInputs inputs) {
        BaseStatusSignal.refreshAll(
            _yaw, _pitch, _roll,
            _yawVelocity, _pitchVelocity, _rollVelocity
        );

        inputs.yawPosition = Rotation2d.fromDegrees(_yaw.getValueAsDouble());
        inputs.pitchPosition = Rotation2d.fromDegrees(_pitch.getValueAsDouble());
        inputs.rollPosition = Rotation2d.fromDegrees(_roll.getValueAsDouble());

        inputs.yawVelocityRadPerSec = Units.degreesToRadians(_yawVelocity.getValueAsDouble());
        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(_pitchVelocity.getValueAsDouble());
        inputs.rollVelocityRadPerSec = Units.degreesToRadians(_rollVelocity.getValueAsDouble());
    }

    @Override
    public void writeOutputs(DriveOutputs outputs) {
        // No outputs,  gyro is read-only
    }

    @Override
    public void reset() {
        _imu.setYaw(0);
    }
}