package org.frc5687.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class HardwareIntakeIO implements IntakeIO{

    private final IntakeConfig _config;
    private final TalonFX _rollerMotor;

    private final StatusSignal<AngularVelocity> _rollerVelocity;

    private final VoltageOut _rollerVoltageReq = new VoltageOut(0);
    public HardwareIntakeIO(int rollerMotorID, IntakeConfig config){
        _rollerMotor = new TalonFX(rollerMotorID, config.canBUS());
        _config = config;
        _rollerVelocity = _rollerMotor.getVelocity();
        configureRollerMotor(config);
    }
    @Override
    public void updateInputs(IntakeInputs inputs) {
        _rollerVelocity.refresh();

        inputs.motorVelocityRadperSec = Units.rotationsToRadians(_rollerVelocity.getValueAsDouble());
        inputs.motorTemperatureCelsius = _rollerMotor.getDeviceTemp().getValueAsDouble();

    }

    @Override
    public void writeOutputs(IntakeOutputs Outputs) {
       
        _rollerMotor.setControl(_rollerVoltageReq.withOutput(Outputs.rollerVoltage));
    }
    

    private void configureRollerMotor(IntakeConfig config){
        var talonConfigs = new TalonFXConfiguration();

        talonConfigs.MotorOutput.NeutralMode = config.isRollerBrakeMode() ?
            NeutralModeValue.Brake:
            NeutralModeValue.Coast;

        talonConfigs.MotorOutput.Inverted = config.isRollerMotorInverted() ?
            InvertedValue.Clockwise_Positive:
            InvertedValue.CounterClockwise_Positive;
        
        talonConfigs.CurrentLimits.SupplyCurrentLimit = config.rollerMotorCurrentLimit();
        talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        _rollerMotor.getConfigurator().apply(talonConfigs);
    }
}
