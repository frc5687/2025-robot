package org.frc5687.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.frc5687.robot.Constants;
import org.frc5687.robot.util.CTREUtil;

public class HardwareElevatorIO implements ElevatorIO {
    private final TalonFX _eastMotor;
    private final TalonFX _westMotor;

    private final StatusSignal<AngularVelocity> _westVelocity;
    private final StatusSignal<Angle> _westPosition;
    private final StatusSignal<AngularVelocity> _eastVelocity;
    private final StatusSignal<Angle> _eastPosition;

    private final MotionMagicExpoTorqueCurrentFOC _westPositionTorqueRequest;
    private final MotionMagicExpoTorqueCurrentFOC _eastPositionTorqueRequest;

    private double _platformVelocity = 0.0;

    public HardwareElevatorIO(int eastMotorId, int westMotorId) {
        _eastMotor = new TalonFX(eastMotorId, Constants.Elevator.CANBUS);
        _westMotor = new TalonFX(westMotorId, Constants.Elevator.CANBUS);

        _westVelocity = _westMotor.getVelocity();
        _westPosition = _westMotor.getPosition();
        _eastVelocity = _eastMotor.getVelocity();
        _eastPosition = _eastMotor.getPosition();

        _westPositionTorqueRequest = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);
        _eastPositionTorqueRequest = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);

        setSignalFrequency();
        setControlFrequency();

        configureMotor(_eastMotor, Constants.Elevator.EAST_INVERTED);
        configureMotor(_westMotor, Constants.Elevator.WEST_INVERTED);
    }

    private void setControlFrequency() {
        _westPositionTorqueRequest.UpdateFreqHz = 1000;
        _eastPositionTorqueRequest.UpdateFreqHz = 1000;
        _westPositionTorqueRequest.UpdateFreqHz = 1000;
    }

    private void setSignalFrequency() {
        _westPosition.setUpdateFrequency(1.0 / Constants.UPDATE_PERIOD);
        _westVelocity.setUpdateFrequency(1.0 / Constants.UPDATE_PERIOD);

        _eastPosition.setUpdateFrequency(1.0 / Constants.UPDATE_PERIOD);
        _eastVelocity.setUpdateFrequency(1.0 / Constants.UPDATE_PERIOD);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        StatusSignal.waitForAll(.2, _westVelocity, _westPosition, _eastVelocity, _eastPosition);

        double eastPosition =
                Units.rotationsToRadians(_eastPosition.getValueAsDouble())
                        * Constants.Elevator.DRUM_RADIUS
                        / Constants.Elevator.GEAR_RATIO;

        double westPosition =
                Units.rotationsToRadians(_westPosition.getValueAsDouble())
                        * Constants.Elevator.DRUM_RADIUS
                        / Constants.Elevator.GEAR_RATIO;

        inputs.heightPositionMeters = (eastPosition + westPosition) / 2.0;

        _platformVelocity =
                (_eastVelocity.getValueAsDouble() + _westVelocity.getValueAsDouble() / 2.0)
                        * (2 * Math.PI * Constants.Elevator.DRUM_RADIUS)
                        / Constants.Elevator.GEAR_RATIO;
        inputs.firstStageVelocityMPS = _platformVelocity;
        inputs.platformMotorCurrents =
                new double[] {
                    _eastMotor.getSupplyCurrent().getValueAsDouble(),
                    _westMotor.getSupplyCurrent().getValueAsDouble(),
                };
    }

    @Override
    public void writeOutputs(ElevatorOutputs outputs) {
        // double nwPos = outputs.desiredStageHeight + (outputs.desiredStageHeight *
        // backlashOffset.get());
        // double nePos = outputs.desiredStageHeight + (outputs.desiredStageHeight *
        // backlashOffset.get());
        // double swPos = outputs.desiredStageHeight - (outputs.desiredStageHeight *
        // backlashOffset.get());

        // double nwRotations =
        //         Units.radiansToRotations(outputs.northWestStageHeight /
        // Constants.Elevator.DRUM_RADIUS)
        //                 * Constants.Elevator.GEAR_RATIO_NORTH;

        // double neRotations =
        //         Units.radiansToRotations(outputs.northEastStageHeight /
        // Constants.Elevator.DRUM_RADIUS)
        //                 * Constants.Elevator.GEAR_RATIO_NORTH;
        // double swRotations =
        //         Units.radiansToRotations(outputs.southWestStageHeight /
        // Constants.Elevator.DRUM_RADIUS)
        //                 * Constants.Elevator.GEAR_RATIO_SOUTH;

        // if (isWithinPositionTolerance(outputs.desiredStageHeight)) {
        //     outputs.usingPositionHolding = true;
        //     _northWestElevatorMotor.setControl(_northWestPositionRequest.withPosition(nwRotations));
        //     _northEastElevatorMotor.setControl(_northEastPositionRequest.withPosition(neRotations));
        //     _southWestElevatorMotor.setControl(_southWestPositionRequest.withPosition(swRotations));
        // } else {

        //     outputs.usingPositionHolding = false;

        //
        // _northWestElevatorMotor.setControl(_northWestPositionTorqueRequest.withPosition(nwRotations));
        //
        // _northEastElevatorMotor.setControl(_northEastPositionTorqueRequest.withPosition(neRotations));
        //
        // _southWestElevatorMotor.setControl(_southWestPositionTorqueRequest.withPosition(swRotations));
        // }
        // outputs.voltageCommandNorthEast =
        //         _northEastElevatorMotor.getClosedLoopOutput().getValueAsDouble();
        // outputs.voltageCommandNorthWest =
        //         _northWestElevatorMotor.getClosedLoopOutput().getValueAsDouble();
        // outputs.voltageCommandSouthWest =
        //         _southWestElevatorMotor.getClosedLoopOutput().getValueAsDouble();
    }

    private void configureMotor(TalonFX motor, boolean isInverted) {
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.Voltage.withPeakForwardVoltage(Volts.of(12)).withPeakReverseVoltage(Volts.of(-12));
        double gearRatio = Constants.Elevator.GEAR_RATIO;
        double metersToRotations = (1.0 / (Constants.Elevator.DRUM_RADIUS)) * gearRatio;

        config.MotionMagic.MotionMagicCruiseVelocity = 0;
        config.MotionMagic.MotionMagicExpo_kA = Constants.Elevator.MOTION_MAGIC_EXPO_KA;
        config.MotionMagic.MotionMagicExpo_kV = Constants.Elevator.MOTION_MAGIC_EXPO_KV;

        config.Slot0.kP = Constants.Elevator.kP;
        config.Slot0.kI = Constants.Elevator.kI;
        config.Slot0.kD = Constants.Elevator.kD;
        config.Slot0.kS = Constants.Elevator.kS;
        config.Slot0.kV = Constants.Elevator.kV;
        config.Slot0.kA = Constants.Elevator.kA;
        config.Slot0.kG = Constants.Elevator.kG;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.CURRENT_LIMIT;

        // motor.getConfigurator().apply(config);
        CTREUtil.applyConfiguration(motor, config);
    }

    public void setPID(double kP, double kI, double kD, double kV, double kS, double kA, double kG) {
        Slot1Configs config = new Slot1Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kV = kV;
        config.kA = kA;
        config.kS = kS;
        config.kG = kG;

        CTREUtil.applyConfiguration(_eastMotor, config);
        CTREUtil.applyConfiguration(_westMotor, config);
    }
}
