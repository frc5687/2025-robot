package org.frc5687.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import org.frc5687.robot.Constants;
import org.frc5687.robot.Constants.Motors;
import org.frc5687.robot.RobotStateManager.Geometry;

public class SimElevatorIO implements ElevatorIO {
    private static final double kP = 80.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private final DCMotor _gearBox = Motors.getKrakenX44(1);

    private final PIDController _northWestPIDController;
    private final PIDController _northEastPIDController;
    private final PIDController _southWestPIDController;

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
        _northWestPIDController = new PIDController(kP, kI, kD);
        _northEastPIDController = new PIDController(kP, kI, kD);
        _southWestPIDController = new PIDController(kP, kI, kD);

        _northWestPIDController.setTolerance(0.001);
        _northEastPIDController.setTolerance(0.001);
        _southWestPIDController.setTolerance(0.001);

        double platformMass = Constants.Elevator.MASS / 3.0;
        _platformNorthEastSim =
                new ElevatorSim(
                        _gearBox,
                        Constants.Elevator.GEAR_RATIO_NORTH,
                        platformMass,
                        Constants.Elevator.DRUM_RADIUS,
                        0,
                        Geometry.ELEVATOR_MAX_HEIGHT,
                        true,
                        0);
        _platformNorthWestSim =
                new ElevatorSim(
                        _gearBox,
                        Constants.Elevator.GEAR_RATIO_NORTH,
                        platformMass,
                        Constants.Elevator.DRUM_RADIUS,
                        0,
                        Geometry.ELEVATOR_MAX_HEIGHT,
                        true,
                        0);
        _platformSouthWestSim =
                new ElevatorSim(
                        _gearBox,
                        Constants.Elevator.GEAR_RATIO_SOUTH,
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

        inputs.stageNorthEastPositionMeters = _platformNorthEastSim.getPositionMeters();
        inputs.stageNorthWestPositionMeters = _platformNorthWestSim.getPositionMeters();
        inputs.stageSouthWestPositionMeters = _platformSouthWestSim.getPositionMeters();

        _northEastEncoderSim.setDistance(inputs.stageNorthEastPositionMeters);
        _northWestEncoderSim.setDistance(inputs.stageNorthWestPositionMeters);
        _southWestEncoderSim.setDistance(inputs.stageSouthWestPositionMeters);

        _northEastEncoderSim.setRate(_platformNorthEastSim.getVelocityMetersPerSecond());
        _northWestEncoderSim.setRate(_platformNorthWestSim.getVelocityMetersPerSecond());
        _southWestEncoderSim.setRate(_platformSouthWestSim.getVelocityMetersPerSecond());

        inputs.platformVelocityMPS =
                ((_platformNorthEastSim.getVelocityMetersPerSecond()
                                + _platformNorthWestSim.getVelocityMetersPerSecond()
                                + _platformSouthWestSim.getVelocityMetersPerSecond())
                        / 3.0);

        inputs.platformMotorCurrents[0] = _platformNorthEastSim.getCurrentDrawAmps();
        inputs.platformMotorCurrents[1] = _platformNorthWestSim.getCurrentDrawAmps();
        inputs.platformMotorCurrents[2] = _platformSouthWestSim.getCurrentDrawAmps();
    }

    @Override
    public void writeOutputs(ElevatorOutputs outputs) {
        double batteryVoltage = RobotController.getBatteryVoltage();

        double setpointNE = outputs.northEastStageHeight;
        double setpointNW = outputs.northWestStageHeight;
        double setpointSW = outputs.southWestStageHeight;

        _northEastPIDController.setSetpoint(setpointNE);
        _northWestPIDController.setSetpoint(setpointNW);
        _southWestPIDController.setSetpoint(setpointSW);

        double northEastVoltage =
                _northEastPIDController.calculate(_platformNorthEastSim.getPositionMeters());
        double northWestVoltage =
                _northWestPIDController.calculate(_platformNorthWestSim.getPositionMeters());
        double southWestVoltage =
                _southWestPIDController.calculate(_platformSouthWestSim.getPositionMeters());

        double ffVoltsNE = 0.5 * Math.signum(setpointNE - _platformNorthEastSim.getPositionMeters());
        double ffVoltsNW = 0.5 * Math.signum(setpointNW - _platformNorthWestSim.getPositionMeters());
        double ffVoltsSW = 0.5 * Math.signum(setpointSW - _platformSouthWestSim.getPositionMeters());

        outputs.voltageCommandNorthEast = northEastVoltage;
        outputs.voltageCommandNorthWest = northWestVoltage;
        outputs.voltageCommandSouthWest = southWestVoltage;
        outputs.voltageFeedForwards = new double[] {ffVoltsNE, ffVoltsNW, ffVoltsSW};

        _platformNorthEastSim.setInputVoltage(Math.min(batteryVoltage, northEastVoltage + ffVoltsNE));
        _platformNorthWestSim.setInputVoltage(Math.min(batteryVoltage, northWestVoltage + ffVoltsNW));
        _platformSouthWestSim.setInputVoltage(Math.min(batteryVoltage, southWestVoltage + ffVoltsSW));
    }

    public void setPID(double kP, double kI, double kD, double kV, double kS, double kA, double kG) {}
}
