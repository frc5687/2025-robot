package org.frc5687.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class HardwareIntakeIO implements IntakeIO {

    private final IntakeConfig _rollerConfig;
    private final IntakeConfig _intakeConfig;
    private final TalonFX _rollerMotor;
    private final TalonFX _intakeMotor;

    private final StatusSignal<AngularVelocity> _rollerVelocity;
    private final StatusSignal<AngularVelocity> _intakeVelocity;

    private final VoltageOut _rollerVoltageReq = new VoltageOut(0);
    private final VoltageOut _intakeVoltageReq = new VoltageOut(0);

    public HardwareIntakeIO(
            int rollerMotorID, int intakeMotorID, IntakeConfig rollerConfig, IntakeConfig intakeConfig) {
        _rollerMotor = new TalonFX(rollerMotorID, rollerConfig.canBUS());
        _intakeMotor = new TalonFX(rollerMotorID, intakeConfig.canBUS());

        _rollerConfig = rollerConfig;
        _intakeConfig = intakeConfig;

        _rollerVelocity = _rollerMotor.getVelocity();
        _intakeVelocity = _intakeMotor.getVelocity();

        configureRollerMotor(rollerConfig);
        configureIntakeMotor(intakeConfig);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        _rollerVelocity.refresh();
        _intakeVelocity.refresh();

        inputs.rollerVelocityRadperSec = Units.rotationsToRadians(_rollerVelocity.getValueAsDouble());
        inputs.rollerTemperatureCelsius = _rollerMotor.getDeviceTemp().getValueAsDouble();

        inputs.intakeVelocityRadperSec = Units.rotationsToRadians(_intakeVelocity.getValueAsDouble());
        inputs.intakeTemperatureCelsius = _intakeMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void writeOutputs(IntakeOutputs Outputs) {

        _rollerMotor.setControl(_rollerVoltageReq.withOutput(Outputs.rollerVoltage));
        _intakeMotor.setControl(_intakeVoltageReq.withOutput(Outputs.intakeVoltage));
    }

    private void configureIntakeMotor(IntakeConfig config) {
        var talonConfigs = new TalonFXConfiguration();

        talonConfigs.MotorOutput.NeutralMode =
                config.isRollerBrakeMode() ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        talonConfigs.MotorOutput.Inverted =
                config.isRollerMotorInverted()
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        talonConfigs.CurrentLimits.SupplyCurrentLimit = config.rollerMotorCurrentLimit();
        talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        _intakeMotor.getConfigurator().apply(talonConfigs);
    }

    private void configureRollerMotor(IntakeConfig config) {
        var talonConfigs = new TalonFXConfiguration();

        talonConfigs.MotorOutput.NeutralMode =
                config.isRollerBrakeMode() ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        talonConfigs.MotorOutput.Inverted =
                config.isRollerMotorInverted()
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        talonConfigs.CurrentLimits.SupplyCurrentLimit = config.rollerMotorCurrentLimit();
        talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        _rollerMotor.getConfigurator().apply(talonConfigs);
    }
}
