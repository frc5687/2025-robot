package org.frc5687.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.frc5687.robot.Constants;
import org.frc5687.robot.util.drivers.OutliersTalon;

public class ModuleIOCTRE implements ModuleIO {
    private final OutliersTalon _driveMotor;
    private final OutliersTalon _turnMotor;
    private final CANcoder _encoder;
    
    private final VelocityTorqueCurrentFOC _velocityRequest;
    private final MotionMagicTorqueCurrentFOC _angleRequest;
    
    private final StatusSignal<Angle> _drivePosition;
    private final StatusSignal<AngularVelocity> _driveVelocity;
    private final StatusSignal<Angle> _turnAbsolutePosition;
    private final StatusSignal<AngularVelocity> _turnAngularVelocity;

    public BaseStatusSignal[] _signals = new BaseStatusSignal[4];
    
    public ModuleIOCTRE(int driveID, int turnID, int encoderID, String canBus, double encoderOffset) {
        _driveMotor = new OutliersTalon(driveID, canBus, "Drive");
        _turnMotor = new OutliersTalon(turnID, canBus, "Turn");
        _encoder = new CANcoder(encoderID, canBus);
        
        _velocityRequest = new VelocityTorqueCurrentFOC(0).withOverrideCoastDurNeutral(true);
        _angleRequest = new MotionMagicTorqueCurrentFOC(0).withOverrideCoastDurNeutral(true);
        
        _driveMotor.configure(Constants.SwerveModule.DRIVE_CONFIG);
        _turnMotor.configure(Constants.SwerveModule.STEER_CONFIG);
        
        CANcoderConfiguration CANfig = new CANcoderConfiguration();
        // set units of the CANCoder to radians, with velocity being radians per second
        // CANfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; FIXME I'm not sure how to use the phoenix 6 api for this... if the swerves are off by like 90deg or 180 this could be why --xavier
        CANfig.MagnetSensor.MagnetOffset = encoderOffset;
        CANfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        _encoder.getConfigurator().apply(CANfig);
        configureTurnMotorFeedback(encoderID);
        
        _drivePosition = _driveMotor.getPosition();
        _driveVelocity = _driveMotor.getVelocity();
        _turnAbsolutePosition = _encoder.getAbsolutePosition();
        _turnAngularVelocity = _encoder.getVelocity();

        setupSignalFrequencies();

        _signals[0] = _driveVelocity;
        _signals[1] = _drivePosition;
        _signals[2] = _turnAbsolutePosition;
        _signals[3] = _turnAngularVelocity;
    }
    
    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(_drivePosition, _driveVelocity, _turnAbsolutePosition);
        
        inputs.drivePositionRads = BaseStatusSignal.getLatencyCompensatedValue(_drivePosition, _driveVelocity).in(Units.Radians);
        inputs.driveVelocityRadsPerSec = _driveVelocity.getValue().in(Units.RadiansPerSecond);
        inputs.turnAbsolutePosition = Rotation2d.fromRotations(_turnAbsolutePosition.getValue().in(Units.Rotations));
    }
    
    @Override
    public void runDriveVelocitySetpoint(double velocityRadsPerSec, double ffVolts) {
        _driveMotor.setControl(_velocityRequest.withVelocity(Units.RotationsPerSecond.of(velocityRadsPerSec / (2 * Math.PI))));
    }
    
    @Override
    public void runTurnPositionSetpoint(double angleRads) {
        _turnMotor.setControl(_angleRequest.withPosition(Units.Rotations.of(angleRads / (2 * Math.PI))));
    }
    
    private void configureTurnMotorFeedback(int encoderID) {
        var feedbackConfig = new FeedbackConfigs();
        feedbackConfig.FeedbackRemoteSensorID = encoderID;
        feedbackConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        feedbackConfig.RotorToSensorRatio = Constants.SwerveModule.GEAR_RATIO_STEER;
        _turnMotor.configureFeedback(feedbackConfig);
    }
    
    private void setupSignalFrequencies() {
        _drivePosition.setUpdateFrequency(250);
        _driveVelocity.setUpdateFrequency(250);
        _turnAbsolutePosition.setUpdateFrequency(250);
        _turnAngularVelocity.setUpdateFrequency(250);
    }
}