package org.frc5687.robot.subsystems.groundintake;

import org.frc5687.robot.subsystems.drive.modules.SwerveModuleConfig;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class HardwareGroundIntakeIO implements GroundIntakeIO {

    private final GroundIntakeConfig _config;

    private final TalonFX _rightRoller;
    private final TalonFX _leftRoller;
    private final TalonFX _pivot;

    private final DutyCycleEncoder _pivotAbsEncoder;
    
    private final StatusSignal<AngularVelocity> _leftRollerVelocity;
    private final StatusSignal<AngularVelocity> _rightRollerVelocity;
    
    private final PositionVoltage _pivotVoltageReq = new PositionVoltage(0).withUpdateFreqHz(1000.0);

    private final VoltageOut _rightRollerVoltageReq = new VoltageOut(0);
    private final VoltageOut _leftRollerVoltageReq = new VoltageOut(0);

    public HardwareGroundIntakeIO(
        GroundIntakeConfig config,
        int rightRollerMotorID,
        int leftRollerMotorID,
        int pivotMotorID
    ){
        _config = config;

        _leftRoller = new TalonFX(leftRollerMotorID, _config.canBUS());
        _rightRoller = new TalonFX(rightRollerMotorID, _config.canBUS());
        _pivot = new TalonFX(pivotMotorID, _config.canBUS());

        configureLeftRoller(config);
        configureRightRoller(config);
        configurePivot(config);
        
        _pivotAbsEncoder = new DutyCycleEncoder(0);//FIXME: needs offset
        
        _leftRollerVelocity = _leftRoller.getVelocity();
        _rightRollerVelocity = _rightRoller.getVelocity();

    }


    @Override
    public void updateInputs(GroundIntakeInputs inputs) {
        _rightRollerVelocity.refresh();
        _leftRollerVelocity.refresh();
        

        inputs.rightRollerTemperatureCelsius = _rightRoller.getDeviceTemp().getValueAsDouble();
        inputs.rightRollerVelocity = Units.rotationsToRadians(_rightRollerVelocity.getValueAsDouble());

        inputs.leftRollerTemperatureCelsius = _leftRoller.getDeviceTemp().getValueAsDouble();
        inputs.leftRollerVelocity = Units.rotationsToRadians(_leftRollerVelocity.getValueAsDouble());


        inputs.pivotTemperatureCelsius = _pivot.getDeviceTemp().getValueAsDouble();
        inputs.intakePivotAngle = _pivotAbsEncoder.get() * Math.PI * 2.0;
    }

    @Override
    public void writeOutputs(GroundIntakeOutputs Outputs) {
        
        _leftRoller.setControl(_leftRollerVoltageReq.withOutput(Outputs.leftRollerVoltage));
        _rightRoller.setControl(_rightRollerVoltageReq.withOutput(Outputs.rightRollerVoltage));
        _pivot.setControl(_pivotVoltageReq.withPosition(Outputs.pivotTargetAngle));
    }
    
    private void configureRightRoller(GroundIntakeConfig config) {
        var rightRollerConfigs = new TalonFXConfiguration();
        rightRollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightRollerConfigs.MotorOutput.Inverted = config.rightRollerInverted() ? 
            InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;
        
        rightRollerConfigs.Slot0.kP = config.rightRollerPID().kP();
        rightRollerConfigs.Slot0.kI = config.rightRollerPID().kI();
        rightRollerConfigs.Slot0.kD = config.rightRollerPID().kD();
        rightRollerConfigs.Slot0.kS = config.rightRollerKs();
        rightRollerConfigs.Slot0.kV = config.rightRollerKv();
        rightRollerConfigs.Slot0.kA = config.rightRollerKa();
        
        rightRollerConfigs.CurrentLimits.SupplyCurrentLimit = config.rightRollerCurrentLimit();
        rightRollerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        _rightRoller.getConfigurator().apply(rightRollerConfigs);
    }

    private void configureLeftRoller(GroundIntakeConfig config) {
        var leftRollerConfigs = new TalonFXConfiguration();
        leftRollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftRollerConfigs.MotorOutput.Inverted = config.pivotInverted() ? 
            InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;
        
        leftRollerConfigs.Slot0.kP = config.leftRollerPID().kP();
        leftRollerConfigs.Slot0.kI = config.leftRollerPID().kI();
        leftRollerConfigs.Slot0.kD = config.leftRollerPID().kD();
        leftRollerConfigs.Slot0.kS = config.leftRollerKs();
        leftRollerConfigs.Slot0.kV = config.leftRollerKv();
        leftRollerConfigs.Slot0.kA = config.leftRollerKa();
        
        leftRollerConfigs.CurrentLimits.SupplyCurrentLimit = config.leftRollerCurrentLimit();
        leftRollerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        _leftRoller.getConfigurator().apply(leftRollerConfigs);
    }

    private void configurePivot(GroundIntakeConfig config) {
        var pivotConfigs = new TalonFXConfiguration();
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfigs.MotorOutput.Inverted = config.pivotInverted() ? 
            InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;
        
        pivotConfigs.Slot0.kP = config.pivotPID().kP();
        pivotConfigs.Slot0.kI = config.pivotPID().kI();
        pivotConfigs.Slot0.kD = config.pivotPID().kD();
        pivotConfigs.Slot0.kS = config.pivotKs();
        pivotConfigs.Slot0.kV = config.pivotKv();
        pivotConfigs.Slot0.kA = config.pivotKa();
        
        pivotConfigs.CurrentLimits.SupplyCurrentLimit = config.pivotCurrentLimit();
        pivotConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        _pivot.getConfigurator().apply(pivotConfigs);
    }
}
