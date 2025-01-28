package org.frc5687.robot.subsystems.coralarm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;

public class SimCoralArmIO implements CoralArmIO {
    private final SingleJointedArmSim _armSim;
    private final Encoder _armEncoder;
    private final EncoderSim _armEncoderSim;

    private final ProfiledPIDController _controller;

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
    }

    @Override
    public void updateInputs(CoralInputs inputs) {
        // Update arm sim
        _armSim.update(Constants.UPDATE_PERIOD);

        // Update the raw encoders with the sim positoin
        inputs.angleRads = _armSim.getAngleRads();
        inputs.angularVelocityRadPerSec = _armSim.getVelocityRadPerSec();
        _armEncoderSim.setDistance(inputs.angleRads);
        _armEncoderSim.setRate(inputs.angularVelocityRadPerSec);

        inputs.motorCurrent = _armSim.getCurrentDrawAmps();
        inputs.motorTorque = inputs.motorCurrent * Constants.CoralArm.GEARBOX.KtNMPerAmp;
        inputs.armTorque = inputs.motorCurrent * Constants.CoralArm.GEAR_RATIO;
    }

    @Override
    public void writeOutputs(CoralOutputs outputs) {
        double batteryVoltage = RobotController.getBatteryVoltage();

        _controller.setGoal(outputs.desiredAngleRad);
        outputs.controllerOutput = _controller.calculate(_armSim.getAngleRads());
        _armSim.setInputVoltage(outputs.controllerOutput);
    }
}
