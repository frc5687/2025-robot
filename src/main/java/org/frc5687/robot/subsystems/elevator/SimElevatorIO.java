package org.frc5687.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotStateManager.Geometry;

public class SimElevatorIO implements ElevatorIO {
    private static final double kP = 80.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private final DCMotor _gearBox = DCMotor.getKrakenX60Foc(1);
    private final ProfiledPIDController _northWestPIDController;
    private final ProfiledPIDController _northEastPIDController;
    private final ProfiledPIDController _southWestPIDController;

    private final ElevatorSim _platformNorthEastSim;
    private final ElevatorSim _platformNorthWestSim;
    private final ElevatorSim _platformSouthWestSim;

    private final Encoder _northEastEncoder;
    private final Encoder _northWestEncoder;
    private final Encoder _southWestEncoder;

    private final EncoderSim _northEastEncoderSim;
    private final EncoderSim _northWestEncoderSim;
    private final EncoderSim _southWestEncoderSim;

    public SimElevatorIO() {
        TrapezoidProfile.Constraints constraints =
                new TrapezoidProfile.Constraints(
                        Constants.Elevator.MAX_VELOCITY_MPS, Constants.Elevator.MAX_ACCELERATION_MPSS);

        _northWestPIDController = new ProfiledPIDController(kP, kI, kD, constraints);
        _northEastPIDController = new ProfiledPIDController(kP, kI, kD, constraints);
        _southWestPIDController = new ProfiledPIDController(kP, kI, kD, constraints);

        _northWestPIDController.setTolerance(0.001);
        _northEastPIDController.setTolerance(0.001);
        _southWestPIDController.setTolerance(0.001);

        double platformMass = Constants.Elevator.MASS / 3.0;
        _platformNorthEastSim =
                new ElevatorSim(
                        _gearBox,
                        Constants.Elevator.GEAR_RATIO,
                        platformMass,
                        Constants.Elevator.DRUM_RADIUS,
                        0,
                        Geometry.ELEVATOR_MAX_HEIGHT,
                        true,
                        0);

        _platformNorthWestSim =
                new ElevatorSim(
                        _gearBox,
                        Constants.Elevator.GEAR_RATIO,
                        platformMass,
                        Constants.Elevator.DRUM_RADIUS,
                        0,
                        Geometry.ELEVATOR_MAX_HEIGHT,
                        true,
                        0);

        _platformSouthWestSim =
                new ElevatorSim(
                        _gearBox,
                        Constants.Elevator.GEAR_RATIO,
                        platformMass,
                        Constants.Elevator.DRUM_RADIUS,
                        0,
                        Geometry.ELEVATOR_MAX_HEIGHT,
                        true,
                        0);

        _northWestEncoder = new Encoder(12, 13);
        _northEastEncoder = new Encoder(14, 15);
        _southWestEncoder = new Encoder(16, 17);

        _northWestEncoderSim = new EncoderSim(_northWestEncoder);
        _northEastEncoderSim = new EncoderSim(_northEastEncoder);
        _southWestEncoderSim = new EncoderSim(_southWestEncoder);

        _northWestEncoder.setDistancePerPulse(1.0);
        _northEastEncoder.setDistancePerPulse(1.0);
        _southWestEncoder.setDistancePerPulse(1.0);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        _platformNorthEastSim.update(Constants.Elevator.PERIOD);
        _platformNorthWestSim.update(Constants.Elevator.PERIOD);
        _platformSouthWestSim.update(Constants.Elevator.PERIOD);

        // Get raw stage positions from simulation
        inputs.stageNorthEastPositionMeters = _platformNorthEastSim.getPositionMeters();
        inputs.stageNorthWestPositionMeters = _platformNorthWestSim.getPositionMeters();
        inputs.stageSouthWestPositionMeters = _platformSouthWestSim.getPositionMeters();

        // Update encoders with stage positions
        _northEastEncoderSim.setDistance(inputs.stageNorthEastPositionMeters);
        _northWestEncoderSim.setDistance(inputs.stageNorthWestPositionMeters);
        _southWestEncoderSim.setDistance(inputs.stageSouthWestPositionMeters);

        // Update encoder velocities
        _northWestEncoderSim.setRate(_platformNorthWestSim.getVelocityMetersPerSecond());
        _northEastEncoderSim.setRate(_platformNorthEastSim.getVelocityMetersPerSecond());
        _southWestEncoderSim.setRate(_platformSouthWestSim.getVelocityMetersPerSecond());

        // Set motor currents
        inputs.platformMotorCurrents[0] = _platformNorthEastSim.getCurrentDrawAmps();
        inputs.platformMotorCurrents[1] = _platformNorthWestSim.getCurrentDrawAmps();
        inputs.platformMotorCurrents[2] = _platformSouthWestSim.getCurrentDrawAmps();
    }

    @Override
    public void writeOutputs(ElevatorOutputs outputs) {
        double batteryVoltage = RobotController.getBatteryVoltage();

        double desiredStageHeight = outputs.desiredStageHeight;

        _northWestPIDController.setGoal(desiredStageHeight);
        _northEastPIDController.setGoal(desiredStageHeight);
        _southWestPIDController.setGoal(desiredStageHeight);

        double northWestVoltage =
                _northWestPIDController.calculate(_platformNorthWestSim.getPositionMeters());
        double northEastVoltage =
                _northEastPIDController.calculate(_platformNorthEastSim.getPositionMeters());
        double southWestVoltage =
                _southWestPIDController.calculate(_platformSouthWestSim.getPositionMeters());

        double ffVolts =
                Constants.Elevator.MOTION_kS
                        * Math.signum(desiredStageHeight - _platformNorthWestSim.getPositionMeters());

        outputs.voltageCommandNorthEast = northEastVoltage;
        outputs.voltageCommandNorthWest = northWestVoltage;
        outputs.voltageCommandSouthWest = southWestVoltage;
        outputs.voltageFeedForwards = new double[] {ffVolts, ffVolts, ffVolts};

        _platformNorthEastSim.setInputVoltage(Math.min(batteryVoltage, northEastVoltage + ffVolts));
        _platformNorthWestSim.setInputVoltage(Math.min(batteryVoltage, northWestVoltage + ffVolts));
        _platformSouthWestSim.setInputVoltage(Math.min(batteryVoltage, southWestVoltage + ffVolts));
    }
}
