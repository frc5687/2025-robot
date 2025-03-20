package org.frc5687.robot.subsystems.coralarm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;

public class SimCoralArmIO implements CoralArmIO {
    private final SingleJointedArmSim _armSim;
    private final Encoder _armEncoder;
    private final EncoderSim _armEncoderSim;
    private final ProfiledPIDController _controller;

    private boolean _previouslyInFunnelPosition = false;
    private double _timeEnteredFunnelPosition = 0;
    private static final double SIM_CORAL_DETECTION_DELAY = 2.0;
    private boolean _simulatedCoralDetected = false;
    private double _wheelPosition = 0.0;

    public SimCoralArmIO() {
        _armSim =
                new SingleJointedArmSim(
                        Constants.CoralArm.GEARBOX,
                        Constants.CoralArm.GEAR_RATIO,
                        Constants.CoralArm.MOI_ARM,
                        Constants.CoralArm.ARM_LENGTH,
                        Constants.CoralArm.MIN_ANGLE,
                        Constants.CoralArm.MAX_ANGLE,
                        true,
                        0,
                        0.001,
                        0.0);
        _armEncoder = new Encoder(RobotMap.SIM.DIO.CORAL_ENCODER_A, RobotMap.SIM.DIO.CORAL_ENCODER_B);
        _armEncoderSim = new EncoderSim(_armEncoder);
        _armEncoder.setDistancePerPulse(1);
        TrapezoidProfile.Constraints constraints =
                new TrapezoidProfile.Constraints(
                        Constants.CoralArm.MAX_VELOCITY_RAD_PER_SEC,
                        Constants.CoralArm.MAX_ACCELERATION_RAD_PER_SEC_SQUARED);
        _controller =
                new ProfiledPIDController(
                        Constants.CoralArm.SIM_PID_CONSTANTS.kP(),
                        Constants.CoralArm.SIM_PID_CONSTANTS.kI(),
                        Constants.CoralArm.SIM_PID_CONSTANTS.kD(),
                        constraints);
        _controller.disableContinuousInput();
    }

    @Override
    public void updateInputs(CoralInputs inputs) {
        _armSim.update(Constants.UPDATE_PERIOD);

        inputs.angleRads = _armSim.getAngleRads();
        inputs.angularVelocityRadPerSec = _armSim.getVelocityRadPerSec();
        _armEncoderSim.setDistance(inputs.angleRads);
        _armEncoderSim.setRate(inputs.angularVelocityRadPerSec);
        inputs.motorCurrent = _armSim.getCurrentDrawAmps();
        inputs.motorTorque = inputs.motorCurrent * Constants.CoralArm.GEARBOX.KtNMPerAmp;
        inputs.armTorque = inputs.motorCurrent * Constants.CoralArm.GEAR_RATIO;
        inputs.wheelAngle = _wheelPosition;

        updateSimulatedCoralDetection(inputs);
    }

    private void updateSimulatedCoralDetection(CoralInputs inputs) {
        double currentAngle = _armSim.getAngleRads();
        boolean inFunnelPosition =
                isNearAngle(currentAngle, CoralState.RECEIVE_FROM_FUNNEL.getArmAngle());

        boolean inGroundIntakePosition =
                isNearAngle(currentAngle, CoralState.RECEIVE_FROM_GROUND_INTAKE.getArmAngle());
        boolean inAnyReceivePosition = inFunnelPosition || inGroundIntakePosition;

        if (inAnyReceivePosition && !_previouslyInFunnelPosition) {
            _timeEnteredFunnelPosition = Timer.getFPGATimestamp();
        }

        if (inAnyReceivePosition
                && (Timer.getFPGATimestamp() - _timeEnteredFunnelPosition) > SIM_CORAL_DETECTION_DELAY) {
            _simulatedCoralDetected = true;
        }

        if (isInAnyPlacingPosition(currentAngle)) {
            _simulatedCoralDetected = false;
        }

        _previouslyInFunnelPosition = inAnyReceivePosition;

        inputs.isCoralDetected = _simulatedCoralDetected;
    }

    private boolean isNearAngle(double currentAngle, double targetAngle) {
        return Math.abs(currentAngle - targetAngle) < Units.degreesToRadians(5.0);
    }

    private boolean isInAnyPlacingPosition(double currentAngle) {
        return isNearAngle(currentAngle, CoralState.PLACING_L1.getArmAngle())
                || isNearAngle(currentAngle, CoralState.PLACING.getArmAngle())
                || isNearAngle(currentAngle, CoralState.PLACING_L4.getArmAngle());
    }

    @Override
    public void writeOutputs(CoralOutputs outputs) {
        _controller.setGoal(outputs.desiredAngleRad);
        outputs.controllerOutput = _controller.calculate(_armSim.getAngleRads());
        _armSim.setInputVoltage(outputs.controllerOutput + outputs.dynamicsFF);

        if (!outputs.wheelPositionControl) {
            _wheelPosition += outputs.wheelDutyCycle * 0.1; // TODO: not lazy
        }
    }

    @Override
    public void setPID(double kP, double kI, double kD, double kV, double kS, double kG, double kA) {
        _controller.setPID(kP, kI, kD);
    }
}
