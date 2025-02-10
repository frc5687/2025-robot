package org.frc5687.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.sensors.RevBoreEncoder;

// import org.frc5687.robot.util.CTREUtil;

public class HardwareIntakeIO implements IntakeIO {

    private final TalonFX _pivotMotor;
    private final TalonFX _rollerMotor;
    private final TalonFX _beltMotor;

    private final RevBoreEncoder _encoder;
    private final StatusSignal<AngularVelocity> _rollerVelocity;
    private final StatusSignal<AngularVelocity> _intakeVelocity;

    private final VoltageOut _rollerVoltageReq = new VoltageOut(0);
    private final VoltageOut _intakeVoltageReq = new VoltageOut(0);

    private final MotionMagicVoltage _pivotPositionReq = new MotionMagicVoltage(0);

    public HardwareIntakeIO() {
        _pivotMotor = new TalonFX(RobotMap.CAN.TALONFX.INTAKE_ARM, Constants.Intake.CAN_BUS);
        _rollerMotor = new TalonFX(RobotMap.CAN.TALONFX.INTAKE_ROLLER, Constants.Intake.CAN_BUS);
        _beltMotor = new TalonFX(RobotMap.CAN.TALONFX.INTAKE_BELT, Constants.Intake.CAN_BUS);

        _encoder = new RevBoreEncoder(RobotMap.DIO.INTAKE_ENCODER, 0);
        _rollerVelocity = _rollerMotor.getVelocity();
        _intakeVelocity = _beltMotor.getVelocity();

        _pivotMotor.setPosition(_encoder.getAngle() * Constants.Intake.GEAR_RATIO);
        configureMotor(_rollerMotor, Constants.Intake.ROLLER_INVERTED);
        configureMotor(_beltMotor, Constants.Intake.INTAKE_INVERTED);
        configureMotor(_pivotMotor, Constants.Intake.PIVOT_INVERTED);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        _rollerVelocity.refresh();
        _intakeVelocity.refresh();

        inputs.rollerVelocityRadperSec = Units.rotationsToRadians(_rollerVelocity.getValueAsDouble());
        inputs.rollerTemperatureCelsius = _rollerMotor.getDeviceTemp().getValueAsDouble();

        inputs.angularVelocityRadPerSec = Units.rotationsToRadians(_intakeVelocity.getValueAsDouble());
        inputs.intakeTemperatureCelsius = _beltMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void writeOutputs(IntakeOutputs Outputs) {
        _rollerMotor.setControl(_rollerVoltageReq.withOutput(Outputs.rollerVoltage));
        _beltMotor.setControl(_intakeVoltageReq.withOutput(Outputs.intakeVoltage));
        _pivotMotor.setControl(_pivotPositionReq.withPosition(Outputs.pivotTargetAngle));
    }

    private void configureMotor(TalonFX motor, boolean isInverted) {
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.Voltage.withPeakForwardVoltage(Volts.of(12)).withPeakReverseVoltage(Volts.of(-12));

        config.Slot0.kP = Constants.Intake.kP;
        config.Slot0.kI = Constants.Intake.kI;
        config.Slot0.kD = Constants.Intake.kD;
        config.Slot0.kS = Constants.Intake.kS;
        config.Slot0.kV = Constants.Intake.kV;
        config.Slot0.kA = Constants.Intake.kA;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Intake.CURRENT_LIMIT;

        // CTREUtil.applyConfiguration(motor, config);
    }
}
