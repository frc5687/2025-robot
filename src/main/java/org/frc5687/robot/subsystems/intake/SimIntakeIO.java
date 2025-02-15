package org.frc5687.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;

public class SimIntakeIO implements IntakeIO {
    private final SingleJointedArmSim _armSim;
    private final Encoder _armEncoder;
    private final EncoderSim _armEncoderSim;

    private final ProfiledPIDController _controller;

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

        _controller =
                new ProfiledPIDController(
                        Constants.Intake.SIM_PID_CONSTANTS.kP(),
                        Constants.Intake.SIM_PID_CONSTANTS.kI(),
                        Constants.Intake.SIM_PID_CONSTANTS.kD(),
                        constraints);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        // Update arm sim
        _armSim.update(Constants.UPDATE_PERIOD);

        // Update the raw encoders with the sim positoin
        inputs.armAngleRads = _armSim.getAngleRads();
        inputs.angularVelocityRadPerSec = _armSim.getVelocityRadPerSec();
        _armEncoderSim.setDistance(inputs.armAngleRads);
        _armEncoderSim.setRate(inputs.angularVelocityRadPerSec);

        inputs.intakeCurrentDraw = _armSim.getCurrentDrawAmps();
    }

    @Override
    public void writeOutputs(IntakeOutputs outputs) {
        // double batteryVoltage = RobotController.getBatteryVoltage();

        _controller.setGoal(outputs.desiredAngleRad);
        outputs.controllerOutput = _controller.calculate(_armSim.getAngleRads());
        _armSim.setInputVoltage(outputs.controllerOutput);
    }
}
