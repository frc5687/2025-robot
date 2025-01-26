package org.frc5687.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import org.frc5687.robot.Constants;
import org.frc5687.robot.subsystems.elevator.ElevatorIO;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class HardwareElevatorIO implements ElevatorIO{

    private final TalonFX _northWestElevatorMotor;
    private final TalonFX _northEastElevatorMotor;
    private final TalonFX _southEastElevatorMotor;
    private final StatusSignal<AngularVelocity> _elevatorMotorVelocity;
    private final StatusSignal<Angle> _elevatorMotorPosition;

    private final VoltageOut _elevatorMotorVoltageReq = new VoltageOut(0.0);
    // private final MotionMagicVoltage _elevatorMotorPositionReq = new MotionMagicVoltage(0).withEnableFOC(true);

    private final PositionVoltage _elevatorPositionReq = new PositionVoltage(0).withSlot(0);
    public HardwareElevatorIO(int northWestMotorID, int northEastMotorID, int southEastMotorID){
        _northWestElevatorMotor = new TalonFX(northWestMotorID, Constants.Elevator.CANBUS);
        _northEastElevatorMotor = new TalonFX(northEastMotorID, Constants.Elevator.CANBUS);
        _southEastElevatorMotor = new TalonFX(southEastMotorID, Constants.Elevator.CANBUS);
        _elevatorMotorVelocity = _northEastElevatorMotor.getVelocity();
        _elevatorMotorPosition = _northEastElevatorMotor.getPosition();

        configureNorthEastElevatorMotor();
        configureNorthWestElevatorMotor();
        configureSouthEastElevatorMotor();
        _northEastElevatorMotor.setPosition(0);
        _northWestElevatorMotor.setPosition(0);
        _southEastElevatorMotor.setPosition(0);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        
        _elevatorMotorVelocity.refresh();
        _elevatorMotorPosition.refresh();

        inputs.motorVelocityRadPerSec = Units.rotationsToRadians(_elevatorMotorVelocity.getValueAsDouble());
        inputs.motorPositionRad = Units.rotationsToRadians(_elevatorMotorPosition.getValueAsDouble());
        
    }

    @Override
    public void writeOutputs(ElevatorOutputs Outputs) {
        // _northEastElevatorMotor.setControl(_elevatorMotorVoltageReq.withOutput(Outputs.elevatorVoltage));
        // _northWestElevatorMotor.setControl(_elevatorMotorVoltageReq.withOutput(Outputs.elevatorVoltage));
        // _southEastElevatorMotor.setControl(_elevatorMotorVoltageReq.withOutput(Outputs.elevatorVoltage));
        //_elevatorMotor.setControl(_elevatorMotorPositionReq.withPosition(Outputs.desiredElevatorPositionMeters));


        _northEastElevatorMotor.setControl(_elevatorPositionReq.withPosition(Outputs.desiredElevatorPositionMeters));
        _northWestElevatorMotor.setControl(_elevatorPositionReq.withPosition(Outputs.desiredElevatorPositionMeters));
        _southEastElevatorMotor.setControl(_elevatorPositionReq.withPosition(Outputs.desiredElevatorPositionMeters-(Outputs.desiredElevatorPositionMeters/17.5)));
        System.out.println(_northEastElevatorMotor.getClosedLoopError());
        System.out.println(_northEastElevatorMotor.getClosedLoopOutput());
        // System.out.println(_northEastElevatorMotor.getAppliedControl());

    }

    private void configureNorthWestElevatorMotor(){
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = Constants.Elevator.NORTH_WEST_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        double MPS_TO_RPS = 2.0*Math.PI*Constants.Elevator.DRUM_RADIUS;

        // config.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.MAX_VELOCITY_MPS;
        // config.MotionMagic.MotionMagicAcceleration = Constants.Elevator.MAX_ACCELERATION_MPSS/(MPS_TO_RPS);
        // config.MotionMagic.MotionMagicJerk = Constants.Elevator.MAX_JERK_MPSSS/(MPS_TO_RPS);
        
        config.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));
        
        config.MotionMagic.MotionMagicCruiseVelocity = 50;
        config.MotionMagic.MotionMagicAcceleration = 10;
        config.MotionMagic.MotionMagicJerk = 10;

        config.Slot0.kP = Constants.Elevator.kP;
        config.Slot0.kI = Constants.Elevator.kI;
        config.Slot0.kD = Constants.Elevator.kD;
        config.Slot0.kS = Constants.Elevator.kS;
        config.Slot0.kV = Constants.Elevator.kV;
        config.Slot0.kA = Constants.Elevator.kA;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.CURRENT_LIMIT;

        _northWestElevatorMotor.getConfigurator().apply(config);
    }

    private void configureNorthEastElevatorMotor(){
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = Constants.Elevator.NORTH_EAST_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));
        double MPS_TO_RPS = 2.0*Math.PI*Constants.Elevator.DRUM_RADIUS;

        // config.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.MAX_VELOCITY_MPS/(MPS_TO_RPS);
        // config.MotionMagic.MotionMagicAcceleration = Constants.Elevator.MAX_ACCELERATION_MPSS/(MPS_TO_RPS);
        // config.MotionMagic.MotionMagicJerk = Constants.Elevator.MAX_JERK_MPSSS/(MPS_TO_RPS);

        config.MotionMagic.MotionMagicCruiseVelocity = 50;
        config.MotionMagic.MotionMagicAcceleration = 10;
        config.MotionMagic.MotionMagicJerk =10;


        config.Slot0.kP = Constants.Elevator.kP;
        config.Slot0.kI = Constants.Elevator.kI;
        config.Slot0.kD = Constants.Elevator.kD;
        config.Slot0.kS = Constants.Elevator.kS;
        config.Slot0.kV = Constants.Elevator.kV;
        config.Slot0.kA = Constants.Elevator.kA;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.CURRENT_LIMIT;

        _northEastElevatorMotor.getConfigurator().apply(config);
    }

    private void configureSouthEastElevatorMotor(){
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = Constants.Elevator.SOUTH_EAST_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));
        double MPS_TO_RPS = 2.0*Math.PI*Constants.Elevator.DRUM_RADIUS;

        // config.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.MAX_VELOCITY_MPS/(MPS_TO_RPS);
        // config.MotionMagic.MotionMagicAcceleration = Constants.Elevator.MAX_ACCELERATION_MPSS/(MPS_TO_RPS);
        // config.MotionMagic.MotionMagicJerk = Constants.Elevator.MAX_JERK_MPSSS/(MPS_TO_RPS);
        
        config.MotionMagic.MotionMagicCruiseVelocity = 50;
        config.MotionMagic.MotionMagicAcceleration = 10;
        config.MotionMagic.MotionMagicJerk =10;



        config.Slot0.kP = Constants.Elevator.kP;
        config.Slot0.kI = Constants.Elevator.kI;
        config.Slot0.kD = Constants.Elevator.kD;
        config.Slot0.kS = Constants.Elevator.kS;
        config.Slot0.kV = Constants.Elevator.kV;
        config.Slot0.kA = Constants.Elevator.kA;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.CURRENT_LIMIT;

        _southEastElevatorMotor.getConfigurator().apply(config);
    }
}


