package org.frc5687.robot.subsystems.elevator;

import org.frc5687.robot.Constants;
import org.frc5687.robot.subsystems.elevator.ElevatorIO;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class HardwareElevatorIO implements ElevatorIO{

    private final TalonFX _elevatorMotor;

    private final StatusSignal<AngularVelocity> _elevatorMotorVelocity;
    private final StatusSignal<Angle> _elevatorMotorPosition;



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
        // _elevatorMotor.setPosition(_elevatorMotorPosition.withOutput(Outputs.desiredElevatorPositionMeters));
    }

}
