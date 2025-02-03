package org.frc5687.robot.subsystems.algaearm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;

public class SimAlgaeArmIO implements AlgaeArmIO {
    private final SingleJointedArmSim _armSim;
    private final Encoder _armEncoder;
    private final EncoderSim _armEncoderSim;

    private final ProfiledPIDController _controller;

    public SimAlgaeArmIO() {
        _armSim =
                new SingleJointedArmSim(
                        Constants.AlgaeArm.GEARBOX,
                        Constants.AlgaeArm.GEAR_RATIO,
                        Constants.AlgaeArm.MOI_ARM,
                        Constants.AlgaeArm.ARM_LENGTH,
                        Constants.AlgaeArm.MIN_ANGLE,
                        Constants.AlgaeArm.MAX_ANGLE,
                        true,
                        0,
                        0.001,
                        0.0);

        _armEncoder = new Encoder(RobotMap.SIM.DIO.ALGAE_ENCODER_A, RobotMap.SIM.DIO.ALGAE_ENCODER_B);
        _armEncoderSim = new EncoderSim(_armEncoder);
        _armEncoder.setDistancePerPulse(1);

        TrapezoidProfile.Constraints constraints =
                new TrapezoidProfile.Constraints(
                        Constants.AlgaeArm.MAX_VELOCITY_RAD_PER_SEC,
                        Constants.AlgaeArm.MAX_ACCELERATION_RAD_PER_SEC_SQUARED);

        _controller =
                new ProfiledPIDController(
                        Constants.AlgaeArm.SIM_PID_CONSTANTS.kP(),
                        Constants.AlgaeArm.SIM_PID_CONSTANTS.kI(),
                        Constants.AlgaeArm.SIM_PID_CONSTANTS.kD(),
                        constraints);
    }

    @Override
    public void updateInputs(AlgaeInputs inputs) {
        // Update arm sim
        _armSim.update(Constants.UPDATE_PERIOD);

        // Update the raw encoders with the sim positoin
        inputs.angleRads = _armSim.getAngleRads();
        inputs.angularVelocityRadPerSec = _armSim.getVelocityRadPerSec();
        _armEncoderSim.setDistance(inputs.angleRads);
        _armEncoderSim.setRate(inputs.angularVelocityRadPerSec);

        inputs.motorCurrent = _armSim.getCurrentDrawAmps();
        inputs.motorTorque = inputs.motorCurrent * Constants.AlgaeArm.GEARBOX.KtNMPerAmp;
        inputs.armTorque = inputs.motorCurrent * Constants.AlgaeArm.GEAR_RATIO;
    }

    @Override
    public void writeOutputs(AlgaeOutputs outputs) {
        _controller.setGoal(outputs.desiredAngleRad);
        outputs.controllerOutput = _controller.calculate(_armSim.getAngleRads());
        _armSim.setInputVoltage(outputs.controllerOutput);
    }
}
