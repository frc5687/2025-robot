package org.frc5687.robot.subsystems.lights;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SubsystemIO;
import org.frc5687.robot.util.FieldConstants;

public class LightSubsystem extends OutliersSubsystem<LightInputs, LightOutputs> {
    private final RobotContainer _container;
    private double prevBlink;

    private static final double DUTY_CYCLE = 0.5;
    private static final double BARGE_TARGET_X = 7.4;

    public LightSubsystem(RobotContainer container, SubsystemIO<LightInputs, LightOutputs> io) {
        super(container, io, new LightInputs(), new LightOutputs());
        _container = container;
        prevBlink = 0.0;
    }

    @Override
    protected void processInputs() {
        RobotStateManager.getInstance();
    }

    @Override
    protected void periodic(LightInputs inputs, LightOutputs outputs) {
        // if (_container.getAlgae().isAlgaeDetected()) {
        if (true) {
            double driveX = _container.getDrive().getPose().getX();
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                driveX = FieldConstants.fieldLength - driveX;
            }
            double distance = Math.abs(BARGE_TARGET_X - driveX);
            System.out.println(distance);
            if (distance < 0.05) {
                outputs.desiredState = LightState.FIRE;
            } else if (distance < 1.0) {
                double period = distance * 2.0;
                outputs.desiredState = blink(period, LightState.GREEN, LightState.DARK_GREEN);
            } else {
                outputs.desiredState = LightState.DARK_GREEN;
            }
        } else if (_container.getCoral().isCoralDetected()) {
            outputs.desiredState = LightState.WHITE;
        } else {
            outputs.desiredState = LightState.OFF;
        }
    }

    private LightState blink(double periodSeconds, LightState color, LightState color2) {
        double timestamp = Timer.getFPGATimestamp();
        double diff = timestamp - prevBlink;
        if (diff > periodSeconds) {
            prevBlink = timestamp;
            return color;
        } else if (diff > periodSeconds * DUTY_CYCLE) {
            return color2;
        } else {
            return color;
        }
    }
}
