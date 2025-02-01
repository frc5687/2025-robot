package org.frc5687.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.frc5687.robot.Constants;

public class HardwareElevatorIO implements ElevatorIO {

    private final TalonFX _northWestElevatorMotor;
    private final TalonFX _northEastElevatorMotor;
    private final TalonFX _southWestElevatorMotor;
    private final Pigeon2 _imu;

    private final StatusSignal<AngularVelocity> _northWestVelocity;
    private final StatusSignal<Angle> _northWestPosition;
    private final StatusSignal<AngularVelocity> _northEastVelocity;
    private final StatusSignal<Angle> _northEastPosition;
    private final StatusSignal<AngularVelocity> _southWestVelocity;
    private final StatusSignal<Angle> _southWestPosition;

    private final StatusSignal<Angle> _imuPitch;
    private final StatusSignal<Angle> _imuRoll;

    private final MotionMagicVoltage _northWestMotionRequest;
    private final MotionMagicVoltage _northEastMotionRequest;
    private final MotionMagicVoltage _southWestMotionRequest;

    private final MotionMagicExpoVoltage _northWestExpoMotionRequest;
    private final MotionMagicExpoVoltage _northEastExpoMotionRequest;
    private final MotionMagicExpoVoltage _southWestExpoMotionRequest;

    private final PositionVoltage _northWestPositionRequest;
    private final PositionVoltage _northEastPositionRequest;
    private final PositionVoltage _southWestPositionRequest;

    private final PIDController _pitchController;
    private final PIDController _rollController;

    private final VoltageOut _voltageRequest = new VoltageOut(0).withEnableFOC(true);

    private double _firstStageHeight = 0.0;
    private double _rollOffset = 0.0;
    private double _pitchOffset = 0.0;
    private double _platformVelocity = 0.0;

    public HardwareElevatorIO(
            int northWestMotorID, int northEastMotorID, int southEastMotorID, int imuId) {
        _northWestElevatorMotor = new TalonFX(northWestMotorID, Constants.Elevator.CANBUS);
        _northEastElevatorMotor = new TalonFX(northEastMotorID, Constants.Elevator.CANBUS);
        _southWestElevatorMotor = new TalonFX(southEastMotorID, Constants.Elevator.CANBUS);
        _imu = new Pigeon2(imuId, Constants.Elevator.CANBUS);

        _northWestVelocity = _northWestElevatorMotor.getVelocity();
        _northWestPosition = _northWestElevatorMotor.getPosition();
        _northEastVelocity = _northEastElevatorMotor.getVelocity();
        _northEastPosition = _northEastElevatorMotor.getPosition();
        _southWestVelocity = _southWestElevatorMotor.getVelocity();
        _southWestPosition = _southWestElevatorMotor.getPosition();

        _northWestMotionRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        _northEastMotionRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        _southWestMotionRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

        _northWestExpoMotionRequest = new MotionMagicExpoVoltage(0).withSlot(2).withEnableFOC(true);
        _northEastExpoMotionRequest = new MotionMagicExpoVoltage(0).withSlot(2).withEnableFOC(true);
        _southWestExpoMotionRequest = new MotionMagicExpoVoltage(0).withSlot(2).withEnableFOC(true);

        _northWestPositionRequest = new PositionVoltage(0).withSlot(1).withEnableFOC(true);
        _northEastPositionRequest = new PositionVoltage(0).withSlot(1).withEnableFOC(true);
        _southWestPositionRequest = new PositionVoltage(0).withSlot(1).withEnableFOC(true);

        _imuPitch = _imu.getPitch();
        _imuRoll = _imu.getRoll();

        setSignalFrequency();
        setControlFrequency();

        configureMotor(_northEastElevatorMotor, Constants.Elevator.NORTH_EAST_INVERTED);
        configureMotor(_northWestElevatorMotor, Constants.Elevator.NORTH_WEST_INVERTED);
        configureMotor(_southWestElevatorMotor, Constants.Elevator.SOUTH_EAST_INVERTED);

        _northWestElevatorMotor.setPosition(0);
        _northEastElevatorMotor.setPosition(0);
        _southWestElevatorMotor.setPosition(0);

        _pitchController =
                new PIDController(Constants.Elevator.PITCH_kP, 0.0, Constants.Elevator.PITCH_kD);
        _rollController =
                new PIDController(Constants.Elevator.ROLL_kP, 0.0, Constants.Elevator.ROLL_kD);

        _pitchController.setSetpoint(0.0);
        _rollController.setSetpoint(0.0);

        _pitchController.setTolerance(Units.degreesToRadians(0.1));
        _rollController.setTolerance(Units.degreesToRadians(0.1));

        _pitchOffset = _imuPitch.getValueAsDouble();
        _rollOffset = _imu.getRoll().getValueAsDouble();
    }

    private void setControlFrequency() {
        _northEastPositionRequest.UpdateFreqHz = 1000;
        _northEastMotionRequest.UpdateFreqHz = 1000;

        _northWestPositionRequest.UpdateFreqHz = 1000;
        _northWestMotionRequest.UpdateFreqHz = 1000;

        _southWestPositionRequest.UpdateFreqHz = 1000;
        _southWestMotionRequest.UpdateFreqHz = 1000;
    }

    private void setSignalFrequency() {
        _northWestPosition.setUpdateFrequency(1.0 / Constants.Elevator.PERIOD);
        _northWestVelocity.setUpdateFrequency(1.0 / Constants.Elevator.PERIOD);

        _southWestPosition.setUpdateFrequency(1.0 / Constants.Elevator.PERIOD);
        _southWestVelocity.setUpdateFrequency(1.0 / Constants.Elevator.PERIOD);

        _northEastPosition.setUpdateFrequency(1.0 / Constants.Elevator.PERIOD);
        _northEastVelocity.setUpdateFrequency(1.0 / Constants.Elevator.PERIOD);

        _imuPitch.setUpdateFrequency(1.0 / Constants.Elevator.PERIOD);
        _imuRoll.setUpdateFrequency(1.0 / Constants.Elevator.PERIOD);
    }

    public double[] calculateHeightCorrections(
            double baseHeight, double currentPitch, double currentRoll) {
        double pitchCorrection = _pitchController.calculate(currentPitch);
        double rollCorrection = _rollController.calculate(currentRoll);

        pitchCorrection =
                MathUtil.clamp(
                        pitchCorrection,
                        -Constants.Elevator.MAX_POSITION_CORRECTION,
                        Constants.Elevator.MAX_POSITION_CORRECTION);
        rollCorrection =
                MathUtil.clamp(
                        rollCorrection,
                        -Constants.Elevator.MAX_POSITION_CORRECTION,
                        Constants.Elevator.MAX_POSITION_CORRECTION);

        double northWest = baseHeight - pitchCorrection + rollCorrection;
        double northEast = baseHeight - pitchCorrection - rollCorrection;
        double southWest = baseHeight + pitchCorrection + rollCorrection;

        return new double[] {northWest, northEast, southWest};
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        StatusSignal.waitForAll(
                .2,
                _northWestVelocity,
                _northWestPosition,
                _northEastVelocity,
                _northEastPosition,
                _southWestVelocity,
                _southWestPosition,
                _imuPitch,
                _imuRoll);

        inputs.stageNorthWestPositionMeters =
                Units.rotationsToRadians(_northWestPosition.getValueAsDouble())
                        * Constants.Elevator.DRUM_RADIUS
                        / Constants.Elevator.GEAR_RATIO;
        inputs.stageNorthEastPositionMeters =
                Units.rotationsToRadians(_northEastPosition.getValueAsDouble())
                        * Constants.Elevator.DRUM_RADIUS
                        / Constants.Elevator.GEAR_RATIO;
        inputs.stageSouthWestPositionMeters =
                Units.rotationsToRadians(_southWestPosition.getValueAsDouble())
                        * Constants.Elevator.DRUM_RADIUS
                        / Constants.Elevator.GEAR_RATIO;

        inputs.firstStagePositionMeters =
                (inputs.stageNorthWestPositionMeters
                                + inputs.stageNorthEastPositionMeters
                                + inputs.stageSouthWestPositionMeters)
                        / 3.0;

        inputs.platformPitchRadians = Units.degreesToRadians(getPitch());
        inputs.platformRollRadians = Units.degreesToRadians(getRoll());
        _platformVelocity =
                (_northEastVelocity.getValueAsDouble()
                                + _northWestVelocity.getValueAsDouble()
                                + _southWestVelocity.getValueAsDouble() / 3.0)
                        * (2 * Math.PI * Constants.Elevator.DRUM_RADIUS)
                        / Constants.Elevator.GEAR_RATIO;
        inputs.platformVelocityMPS = _platformVelocity;
        inputs.platformMotorCurrents =
                new double[] {
                    _northWestElevatorMotor.getSupplyCurrent().getValueAsDouble(),
                    _northEastElevatorMotor.getSupplyCurrent().getValueAsDouble(),
                    _southWestElevatorMotor.getSupplyCurrent().getValueAsDouble()
                };

        _firstStageHeight = inputs.firstStagePositionMeters;
    }

    @Override
    public void writeOutputs(ElevatorOutputs outputs) {
        double desiredHeight = outputs.desiredStageHeight;

        // double desiredRotations = Units.radiansToRotations(desiredHeight /
        // Constants.Elevator.DRUM_RADIUS) * Constants.Elevator.GEAR_RATIO;

        // System.out.println(_northEastElevatorMotor.getClosedLoopOutput());
        double[] correctedHeights =
                calculateHeightCorrections(
                        desiredHeight, Units.degreesToRadians(getPitch()), Units.degreesToRadians(getRoll()));

        double nwRotations =
                Units.radiansToRotations(correctedHeights[0] / Constants.Elevator.DRUM_RADIUS)
                        * Constants.Elevator.GEAR_RATIO;
        double neRotations =
                Units.radiansToRotations(correctedHeights[1] / Constants.Elevator.DRUM_RADIUS)
                        * Constants.Elevator.GEAR_RATIO;
        double swRotations =
                Units.radiansToRotations(correctedHeights[2] / Constants.Elevator.DRUM_RADIUS)
                        * Constants.Elevator.GEAR_RATIO;

        double desiredRotations =
                Units.radiansToRotations(desiredHeight)
                        / Constants.Elevator.DRUM_RADIUS
                        * Constants.Elevator.GEAR_RATIO;

        // If we are looking to hold a position, use the more aggressive holding pid including using the
        // pitch controller
        // if (isWithinPositionTolerance(desiredHeight)) {
        //     outputs.usingPositionHolding = true;
        //     _northWestElevatorMotor.setControl(_northWestPositionRequest.withPosition(nwRotations));
        //     _northEastElevatorMotor.setControl(_northEastPositionRequest.withPosition(neRotations));
        //     _southWestElevatorMotor.setControl(_southWestPositionRequest.withPosition(swRotations));
        // } else {
        // Otherwise use motion magic
        outputs.usingPositionHolding = false;
        //
        // _northWestElevatorMotor.setControl(_northWestMotionRequest.withPosition(desiredRotations));
        //
        // _northEastElevatorMotor.setControl(_northEastMotionRequest.withPosition(desiredRotations));
        //
        // _southWestElevatorMotor.setControl(_southWestMotionRequest.withPosition(desiredRotations));

        _northWestElevatorMotor.setControl(_northWestExpoMotionRequest.withPosition(desiredRotations));
        _northEastElevatorMotor.setControl(_northEastExpoMotionRequest.withPosition(desiredRotations));
        _southWestElevatorMotor.setControl(_southWestExpoMotionRequest.withPosition(desiredRotations));
        // }
        outputs.voltageCommandNorthEast =
                _northEastElevatorMotor.getClosedLoopOutput().getValueAsDouble();
        outputs.voltageCommandNorthWest =
                _northWestElevatorMotor.getClosedLoopOutput().getValueAsDouble();
        outputs.voltageCommandSouthWest =
                _southWestElevatorMotor.getClosedLoopOutput().getValueAsDouble();
    }

    private boolean isWithinPositionTolerance(double desiredHeight) {
        return Math.abs(_firstStageHeight - desiredHeight) < Constants.Elevator.MAX_POSITION_CORRECTION;
        // & Math.abs(_platformVelocity) < Constants.Elevator.VELOCITY_TOLERANCE;
    }

    private void configureMotor(TalonFX motor, boolean isInverted) {
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.Voltage.withPeakForwardVoltage(Volts.of(12)).withPeakReverseVoltage(Volts.of(-12));

        double metersToRotations =
                (1.0 / (Constants.Elevator.DRUM_RADIUS)) * Constants.Elevator.GEAR_RATIO;
        // config.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.MAX_VELOCITY_MPS *
        // metersToRotations;
        // config.MotionMagic.MotionMagicAcceleration = Constants.Elevator.MAX_ACCELERATION_MPSS *
        // metersToRotations;
        // config.MotionMagic.MotionMagicJerk = Constants.Elevator.MAX_JERK_MPSSS * metersToRotations;

        config.MotionMagic.MotionMagicCruiseVelocity = 0;
        config.MotionMagic.MotionMagicExpo_kA = Constants.Elevator.MOTION_MAGIC_EXPO_KA;
        config.MotionMagic.MotionMagicExpo_kV = Constants.Elevator.MOTION_MAGIC_EXPO_KV;

        config.Slot0.kP = Constants.Elevator.MOTION_kP;
        config.Slot0.kI = Constants.Elevator.MOTION_kI;
        config.Slot0.kD = Constants.Elevator.MOTION_kD;
        config.Slot0.kS = Constants.Elevator.MOTION_kS;
        config.Slot0.kV = Constants.Elevator.MOTION_kV;
        config.Slot0.kA = Constants.Elevator.MOTION_kA;

        config.Slot1.kP = Constants.Elevator.HOLD_kP;
        config.Slot1.kI = Constants.Elevator.HOLD_kI;
        config.Slot1.kD = Constants.Elevator.HOLD_kD;
        config.Slot1.kS = Constants.Elevator.HOLD_kS;
        config.Slot1.kV = Constants.Elevator.HOLD_kV;
        config.Slot1.kA = Constants.Elevator.HOLD_kA;

        config.Slot2.kP = Constants.Elevator.EXPO_MOTION_kP;
        config.Slot2.kI = Constants.Elevator.EXPO_MOTION_kI;
        config.Slot2.kD = Constants.Elevator.EXPO_MOTION_kD;
        config.Slot2.kS = Constants.Elevator.EXPO_MOTION_kS;
        config.Slot2.kV = Constants.Elevator.EXPO_MOTION_kV;
        config.Slot2.kA = Constants.Elevator.EXPO_MOTION_kA;
        config.Slot2.kG = Constants.Elevator.EXPO_MOTION_kG;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.CURRENT_LIMIT;

        motor.getConfigurator().apply(config);
    }

    public boolean isLevel(double currentPitch, double currentRoll) {
        return _pitchController.atSetpoint() && _rollController.atSetpoint();
    }

    private double getPitch() {
        return _imuPitch.getValueAsDouble() - _pitchOffset;
    }

    private double getRoll() {
        return _imuRoll.getValueAsDouble() - _rollOffset;
    }
}
