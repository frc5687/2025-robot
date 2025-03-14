package org.frc5687.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Servo;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.sensors.ProximitySensor;

public class HardwareClimberArmIO implements ClimberIO {
    private final TalonFX _winchMotor;
    private final StatusSignal<Angle> _winchAngle;
    private final StatusSignal<Current> _supplyCurrent;
    private final StatusSignal<Current> _statorCurrent;
    private final VoltageOut _voltageRequest;
    private final ProximitySensor _sensor;
    private final Servo _servo;

    public HardwareClimberArmIO() {
        _winchMotor = new TalonFX(RobotMap.CAN.TALONFX.CLIMBER_WINCH, Constants.Climber.CAN_BUS);
        _winchMotor.setPosition(0);
        _servo = new Servo(RobotMap.PWM.CLIMBER_SERVO);
        _winchAngle = _winchMotor.getPosition();
        _supplyCurrent = _winchMotor.getSupplyCurrent();
        _statorCurrent = _winchMotor.getStatorCurrent();
        _voltageRequest = new VoltageOut(0.0).withEnableFOC(true);
        _sensor = new ProximitySensor(RobotMap.DIO.CLIMBER_SENSOR);

        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 120;
        _winchMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        StatusSignal.refreshAll(_winchAngle, _statorCurrent, _supplyCurrent);
        inputs.motorAngleRads = _winchAngle.getValue().in(Radians);
        inputs.statorCurrent = _statorCurrent.getValue().in(Amps);
        inputs.supplyCurrent = _supplyCurrent.getValue().in(Amps);
        inputs.sensor = _sensor.get();
    }

    @Override
    public void writeOutputs(ClimberOutputs outputs) {
        _winchMotor.setControl(_voltageRequest.withOutput(outputs.climberVoltage));
        _servo.set(outputs.servoSetpoint);
    }
}
