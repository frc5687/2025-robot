package org.frc5687.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import org.frc5687.robot.Constants;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class HardwareElevatorIO implements ElevatorIO {
    private final TalonFX _northWestElevatorMotor;
    private final TalonFX _northEastElevatorMotor;
    private final TalonFX _southEastElevatorMotor;
    
    private final StatusSignal<AngularVelocity> _northWestVelocity;
    private final StatusSignal<Angle> _northWestPosition;
    private final StatusSignal<AngularVelocity> _northEastVelocity;
    private final StatusSignal<Angle> _northEastPosition;
    private final StatusSignal<AngularVelocity> _southEastVelocity;
    private final StatusSignal<Angle> _southEastPosition;

    private final MotionMagicVoltage _northWestPositionRequest;
    private final MotionMagicVoltage _northEastPositionRequest;
    private final MotionMagicVoltage _southEastPositionRequest;

    private final VoltageOut _voltageRequest = new VoltageOut(0).withEnableFOC(true);

    public HardwareElevatorIO(int northWestMotorID, int northEastMotorID, int southEastMotorID) {
        _northWestElevatorMotor = new TalonFX(northWestMotorID, Constants.Elevator.CANBUS);
        _northEastElevatorMotor = new TalonFX(northEastMotorID, Constants.Elevator.CANBUS);
        _southEastElevatorMotor = new TalonFX(southEastMotorID, Constants.Elevator.CANBUS);

        _northWestVelocity = _northWestElevatorMotor.getVelocity();
        _northWestPosition = _northWestElevatorMotor.getPosition();
        _northEastVelocity = _northEastElevatorMotor.getVelocity();
        _northEastPosition = _northEastElevatorMotor.getPosition();
        _southEastVelocity = _southEastElevatorMotor.getVelocity();
        _southEastPosition = _southEastElevatorMotor.getPosition();

        _northWestPositionRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        _northEastPositionRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        _southEastPositionRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

        configureMotor(_northEastElevatorMotor, Constants.Elevator.NORTH_EAST_INVERTED);
        configureMotor(_northWestElevatorMotor, Constants.Elevator.NORTH_WEST_INVERTED);
        configureMotor(_southEastElevatorMotor, Constants.Elevator.SOUTH_EAST_INVERTED);

        _northWestElevatorMotor.setPosition(0);
        _northEastElevatorMotor.setPosition(0);
        _southEastElevatorMotor.setPosition(0);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        StatusSignal.waitForAll(.2,
            _northWestVelocity,
            _northWestPosition,
            _northEastVelocity,
            _northEastPosition,
            _southEastVelocity,
            _southEastPosition
        );

        inputs.stageNorthWestPositionMeters = Units.rotationsToRadians(_northWestPosition.getValueAsDouble()) 
            * Constants.Elevator.DRUM_RADIUS / Constants.Elevator.GEAR_RATIO;
        inputs.stageNorthEastPositionMeters = Units.rotationsToRadians(_northEastPosition.getValueAsDouble()) 
            * Constants.Elevator.DRUM_RADIUS / Constants.Elevator.GEAR_RATIO;
        inputs.stageSouthWestPositionMeters = Units.rotationsToRadians(_southEastPosition.getValueAsDouble()) 
            * Constants.Elevator.DRUM_RADIUS / Constants.Elevator.GEAR_RATIO;

        inputs.platformMotorCurrents = new double[]{
            _northWestElevatorMotor.getSupplyCurrent().getValueAsDouble(),
            _northEastElevatorMotor.getSupplyCurrent().getValueAsDouble(),
            _southEastElevatorMotor.getSupplyCurrent().getValueAsDouble()
        };
    }

    @Override
    public void writeOutputs(ElevatorOutputs outputs) {
        double desiredHeight = outputs.desiredStageHeight;
        double desiredRotations = Units.radiansToRotations(desiredHeight / Constants.Elevator.DRUM_RADIUS) * Constants.Elevator.GEAR_RATIO;  

        // System.out.println(_northEastElevatorMotor.getClosedLoopOutput());
        _northWestElevatorMotor.setControl(_northWestPositionRequest.withPosition(desiredRotations));
        _northEastElevatorMotor.setControl(_northEastPositionRequest.withPosition(desiredRotations));
        _southEastElevatorMotor.setControl(_southEastPositionRequest.withPosition(desiredRotations - 0.15)); // make this real
    }

    private void configureMotor(TalonFX motor, boolean isInverted) {
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = isInverted ? 
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.Voltage.withPeakForwardVoltage(Volts.of(12))
            .withPeakReverseVoltage(Volts.of(-12));

        double metersToRotations = 1.0 / (2.0 * Math.PI * Constants.Elevator.DRUM_RADIUS);
        
        config.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.MAX_VELOCITY_MPS * metersToRotations;
        config.MotionMagic.MotionMagicAcceleration = Constants.Elevator.MAX_ACCELERATION_MPSS * metersToRotations;
        config.MotionMagic.MotionMagicJerk = Constants.Elevator.MAX_JERK_MPSSS * metersToRotations;

        config.Slot0.kP = Constants.Elevator.kP;
        config.Slot0.kI = Constants.Elevator.kI;
        config.Slot0.kD = Constants.Elevator.kD;
        config.Slot0.kS = Constants.Elevator.kS;
        config.Slot0.kV = Constants.Elevator.kV;
        config.Slot0.kA = Constants.Elevator.kA;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.CURRENT_LIMIT;

        motor.getConfigurator().apply(config);
    }
}