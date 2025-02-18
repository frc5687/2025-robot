package org.frc5687.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import org.frc5687.robot.Constants;

public class SimElevatorIO implements ElevatorIO {
    private static final double kP = 80.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private final ProfiledPIDController _elevatorPIDController;
    private final ElevatorSim _elevatorSim;
    private final Encoder _elevatorEncoder;
    private final EncoderSim _elevatorEncoderSim;

    public SimElevatorIO() {
        _elevatorPIDController = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(
                Constants.Elevator.MAX_VELOCITY_MPS,
                Constants.Elevator.MAX_ACCELERATION_MPSS
        ));

        _elevatorPIDController.setTolerance(0.01);

        _elevatorSim =
                new ElevatorSim(
                        Constants.Elevator.MOTOR,
                        Constants.Elevator.GEAR_RATIO,
                        Constants.Elevator.MASS,
                        Constants.Elevator.DRUM_RADIUS,
                        0,
                        Constants.Elevator.MAX_PLATFORM_HEIGHT / 2.0,
                        true,
                        0,
                        0.0001, 0.0001);

        _elevatorEncoder = new Encoder(16, 17);
        _elevatorEncoderSim = new EncoderSim(_elevatorEncoder);
        _elevatorEncoder.setDistancePerPulse(1.0);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        _elevatorSim.update(Constants.UPDATE_PERIOD);

        inputs.firstStagePositionMeters = _elevatorSim.getPositionMeters();
        inputs.firstStageVelocityMPS = _elevatorSim.getVelocityMetersPerSecond();

        _elevatorEncoderSim.setDistance(inputs.firstStagePositionMeters);
        _elevatorEncoderSim.setRate(_elevatorSim.getVelocityMetersPerSecond());

        inputs.platformMotorCurrents[0] = _elevatorSim.getCurrentDrawAmps();
        inputs.platformMotorCurrents[1] = _elevatorSim.getCurrentDrawAmps();
    }

    @Override
    public void writeOutputs(ElevatorOutputs outputs) {
        double batteryVoltage = RobotController.getBatteryVoltage();

        double currentPosition = _elevatorSim.getPositionMeters();

        double pidVolts = _elevatorPIDController.calculate(currentPosition, outputs.desiredStageHeight);

        double ffVolts =
                0.5 * Math.signum(outputs.desiredStageHeight - _elevatorSim.getPositionMeters());
        double totalVoltage = Math.min(batteryVoltage, Math.max(-batteryVoltage, ffVolts + pidVolts));

        outputs.voltageCommandEast = ffVolts + pidVolts;
        outputs.voltageCommandWest = ffVolts + pidVolts;

        _elevatorSim.setInputVoltage(totalVoltage);
    }

    @Override
    public void setPID(double kP, double kI, double kD, double kV, double kS, double kA, double kG) {
        _elevatorPIDController.setPID(kP, kI, kD);
    }
}
