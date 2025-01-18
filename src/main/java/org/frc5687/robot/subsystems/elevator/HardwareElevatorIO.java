package org.frc5687.robot.subsystems.elevator;

import org.frc5687.robot.Constants;
import org.frc5687.robot.subsystems.elevator.ElevatorIO;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class HardwareElevatorIO implements ElevatorIO{

    private final TalonFX _elevatorMotor;

    private final StatusSignal<AngularVelocity> _elevatorMotorVelocity;
    private final StatusSignal<Angle> _elevatorMotorPosition;

    private final VoltageOut _elevatorMotorVoltageReq = new VoltageOut(0.0);
    private final MotionMagicVoltage _elevatorMotorPositionReq = new MotionMagicVoltage(0).withEnableFOC(true);

    public HardwareElevatorIO(int elevatormotorID){
        _elevatorMotor = new TalonFX(elevatormotorID, Constants.Elevator.CANBUS);
        
        _elevatorMotorVelocity = _elevatorMotor.getVelocity();
        _elevatorMotorPosition = _elevatorMotor.getPosition();

        
        

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
        _elevatorMotor.setControl(_elevatorMotorVoltageReq.withOutput(Outputs.elevatorVoltage));

        _elevatorMotor.setControl(_elevatorMotorPositionReq.withPosition(Outputs.desiredElevatorPositionMeters));
    }

    private void configureElevatorMotor(){
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = Constants.Elevator.INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        double MPS_TO_RPS = 2.0*Math.PI*Constants.Elevator.DRUM_RADIUS;

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.MAX_VELOCITY_MPS/(MPS_TO_RPS);
        config.MotionMagic.MotionMagicAcceleration = Constants.Elevator.MAX_ACCELERATION_MPSS/(MPS_TO_RPS);
        config.MotionMagic.MotionMagicJerk = Constants.Elevator.MAX_JERK_MPSSS/(MPS_TO_RPS);
        


        config.Slot0.kP = Constants.Elevator.kP;
        config.Slot0.kP = Constants.Elevator.kI;
        config.Slot0.kP = Constants.Elevator.kD;
        config.Slot0.kP = Constants.Elevator.kS;
        config.Slot0.kP = Constants.Elevator.kV;
        config.Slot0.kP = Constants.Elevator.kA;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.CURRENT_LIMIT;

        _elevatorMotor.getConfigurator().apply(config);
    }
}
