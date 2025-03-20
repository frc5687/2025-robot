package org.frc5687.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;

public class SimIntakeIO implements IntakeIO {
    private final SingleJointedArmSim _armSim;
    private final Encoder _armEncoder;
    private final EncoderSim _armEncoderSim;
    private final ProfiledPIDController _profiledPIDController;

    private double _rollerVoltage = 0.0;
    private double _intakeVoltage = 0.0;

    private boolean _previouslyInIntakePosition = false;
    private double _timeEnteredIntakePosition = 0;
    private static final double SIM_CORAL_DETECTION_DELAY = 1.0;
    private boolean _simulatedCoralDetected = false;

    public SimIntakeIO() {
        _armSim =
                new SingleJointedArmSim(
                        Constants.Intake.GEARBOX,
                        Constants.Intake.GEAR_RATIO,
                        Constants.Intake.MOI_ARM,
                        Constants.Intake.ARM_LENGTH,
                        Constants.Intake.MIN_ANGLE,
                        Constants.Intake.MAX_ANGLE,
                        true,
                        0,
                        0.001,
                        0.0);

        _armEncoder = new Encoder(RobotMap.SIM.DIO.INTAKE_ENCODER_A, RobotMap.SIM.DIO.INTAKE_ENCODER_B);
        _armEncoderSim = new EncoderSim(_armEncoder);
        _armEncoder.setDistancePerPulse(1);

        TrapezoidProfile.Constraints constraints =
                new TrapezoidProfile.Constraints(
                        Constants.Intake.MAX_VELOCITY_RAD_PER_SEC,
                        Constants.Intake.MAX_ACCELERATION_RAD_PER_SEC_SQUARED);

        _profiledPIDController =
                new ProfiledPIDController(
                        Constants.Intake.SIM_PID_CONSTANTS.kP(),
                        Constants.Intake.SIM_PID_CONSTANTS.kI(),
                        Constants.Intake.SIM_PID_CONSTANTS.kD(),
                        constraints);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        _armSim.update(Constants.UPDATE_PERIOD);

        double position = _armSim.getAngleRads();
        double velocity = _armSim.getVelocityRadPerSec();

        _armEncoderSim.setDistance(position);
        _armEncoderSim.setRate(velocity);

        inputs.armAngleRads = position;

        updateSimulatedCoralDetection(inputs);
    }

    @Override
    public void writeOutputs(IntakeOutputs outputs) {
        _rollerVoltage = outputs.rollerVoltage;
        _intakeVoltage = outputs.intakeVoltage;

        _profiledPIDController.setGoal(outputs.desiredAngleRad);
        double voltage = _profiledPIDController.calculate(_armSim.getAngleRads());
        voltage += outputs.dynamicsFF;

        _armSim.setInputVoltage(voltage);

        if (_rollerVoltage < -5.0) {
            _simulatedCoralDetected = false;
        }
    }

    private void updateSimulatedCoralDetection(IntakeInputs inputs) {
        double currentAngle = _armSim.getAngleRads();

        boolean inIntakePosition = isNearAngle(currentAngle, IntakeState.DEPLOYED.getValue());

        if (inIntakePosition && !_previouslyInIntakePosition) {
            _timeEnteredIntakePosition = Timer.getFPGATimestamp();
        }

        if (inIntakePosition
                && (Timer.getFPGATimestamp() - _timeEnteredIntakePosition) > SIM_CORAL_DETECTION_DELAY) {
            _simulatedCoralDetected = true;
        }

        if (isNearAngle(currentAngle, IntakeState.PASSOFF_TO_CORAL.getValue())
                || isNearAngle(currentAngle, IntakeState.IDLE.getValue())) {
            _simulatedCoralDetected = false;
        }

        _previouslyInIntakePosition = inIntakePosition;
        inputs.isCoralDetected = _simulatedCoralDetected;
    }

    private boolean isNearAngle(double currentAngle, double targetAngle) {
        return Math.abs(currentAngle - targetAngle) < Units.degreesToRadians(10.0);
    }

    public void setCoralDetected(boolean detected) {
        _simulatedCoralDetected = detected;
    }
}
