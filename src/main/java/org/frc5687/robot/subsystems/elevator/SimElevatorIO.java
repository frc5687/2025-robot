package org.frc5687.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotStateManager.Geometry;

public class SimElevatorIO implements ElevatorIO {
    private double _kP = 200.0;
    private double _kI = 0.0;
    private double _kD = 0.0;

    private double _kS = Constants.Elevator.kS;
    private double _kV = Constants.Elevator.kV;
    private double _kA = Constants.Elevator.kA;
    private double _kG = Constants.Elevator.kG;

    private final ProfiledPIDController _elevatorPIDController;
    private SimpleMotorFeedforward _feedforward;
    private final ElevatorSim _elevatorSim;
    private final Encoder _elevatorEncoder;
    private final EncoderSim _elevatorEncoderSim;

    public SimElevatorIO() {
        _elevatorPIDController =
                new ProfiledPIDController(
                        _kP,
                        _kI,
                        _kD,
                        new TrapezoidProfile.Constraints(
                                Constants.Elevator.MAX_VELOCITY_MPS, Constants.Elevator.MAX_ACCELERATION_MPSS));

        System.out.println("Elevator Vel MPS: " + Constants.Elevator.MAX_VELOCITY_MPS);
        System.out.println("Elevator Accel: " + Constants.Elevator.MAX_ACCELERATION_MPSS);
        _elevatorPIDController.setTolerance(0.01);

        _feedforward = new SimpleMotorFeedforward(_kS, _kV, _kA);

        _elevatorSim =
                new ElevatorSim(
                        Constants.Elevator.MOTOR,
                        Constants.Elevator.GEAR_RATIO,
                        Constants.Elevator.MASS,
                        Constants.Elevator.DRUM_RADIUS,
                        0.0,
                        Geometry.ELEVATOR_STAGE_ONE_MAX_HEIGHT - Geometry.ELEVATOR_STAGE_ONE_HEIGHT,
                        true,
                        0,
                        0.0001,
                        0.0001);

        _elevatorEncoder = new Encoder(16, 17);
        _elevatorEncoderSim = new EncoderSim(_elevatorEncoder);
        _elevatorEncoder.setDistancePerPulse(1.0);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        _elevatorSim.update(Constants.UPDATE_PERIOD);

        inputs.heightPositionMeters = _elevatorSim.getPositionMeters();
        inputs.firstStageVelocityMPS = _elevatorSim.getVelocityMetersPerSecond();

        _elevatorEncoderSim.setDistance(inputs.heightPositionMeters);
        _elevatorEncoderSim.setRate(_elevatorSim.getVelocityMetersPerSecond());

        double currentDraw = _elevatorSim.getCurrentDrawAmps();
        inputs.platformMotorCurrents[0] = currentDraw;
        inputs.platformMotorCurrents[1] = currentDraw;
    }

    @Override
    public void writeOutputs(ElevatorOutputs outputs) {
        double batteryVoltage = RobotController.getBatteryVoltage();
        double currentPosition = _elevatorSim.getPositionMeters();

        TrapezoidProfile.State setpoint = _elevatorPIDController.getSetpoint();

        double pidOutput = _elevatorPIDController.calculate(currentPosition, outputs.desiredHeight);

        double ffOutput = _feedforward.calculate(setpoint.velocity) + _kG;

        double totalVoltage = pidOutput + ffOutput;
        totalVoltage = Math.max(-batteryVoltage, Math.min(batteryVoltage, totalVoltage));

        outputs.voltageCommandEast = totalVoltage;
        outputs.voltageCommandWest = totalVoltage;

        _elevatorSim.setInputVoltage(totalVoltage);
    }

    @Override
    public void setPID(double kP, double kI, double kD, double kV, double kS, double kA, double kG) {
        this._kP = kP;
        this._kI = kI;
        this._kD = kD;
        this._kV = kV;
        this._kS = kS;
        this._kA = kA;
        this._kG = kG;
        _elevatorPIDController.setPID(kP, kI, kD);
        _feedforward = new SimpleMotorFeedforward(_kS, _kV, _kA);
    }
}
