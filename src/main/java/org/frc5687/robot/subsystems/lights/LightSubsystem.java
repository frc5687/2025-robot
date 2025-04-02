package org.frc5687.robot.subsystems.lights;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SubsystemIO;
import org.frc5687.robot.util.FieldConstants;
import org.frc5687.robot.util.vision.CoralTracker;

public class LightSubsystem extends OutliersSubsystem<LightInputs, LightOutputs> {
    private final RobotContainer _container;

    public LightSubsystem(RobotContainer container, SubsystemIO<LightInputs, LightOutputs> io) {
        super(container, io, new LightInputs(), new LightOutputs());
        _container = container;
    }

    @Override
    protected void processInputs() {
        RobotStateManager.getInstance();
    }

    @Override
    protected void periodic(LightInputs inputs, LightOutputs outputs) {
        if (_container.getClimber().isSensorTriggered()) {
            outputs.desiredState = LightState.BLUE;
        } else if (_container.getIntake().isIntakeCoralDetected()) {
            outputs.desiredState = LightState.PINK;
        } else if (_container.getSuperstructureManager().isAlgaeMode()) {
            if (_container.getAlgae().isAlgaeDetected()) {
                outputs.desiredState = LightState.FLASHING_GREEN;
            } else {
                outputs.desiredState = LightState.SOLID_GREEN;
            }
        } else { // coral mode
            if (_container.getCoral().isCoralDetected()) {
                outputs.desiredState = LightState.FLASHING_WHITE;
            } else {
                var pose = RobotStateManager.getInstance().getPose(RobotCoordinate.ROBOT_BASE_SWERVE);
                if (pose != null
                        && CoralTracker.getInstance().getClosestCoral(pose.toPose2d()).isPresent()) {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isEmpty()) {
                        outputs.desiredState = LightState.YELLOW;
                    } else {
                        outputs.desiredState = LightState.BLUE;
                    }
                } else {
                    outputs.desiredState = LightState.SOLID_WHITE;
                }
            }
        }
    }

    private boolean withinNetTolerance() {
        double targetX = FieldConstants.fieldLength / 2.0 - Units.inchesToMeters(46);

        double fieldX =
                RobotStateManager.getInstance().getPose(RobotCoordinate.ROBOT_BASE_SWERVE).getX();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            fieldX = FieldConstants.fieldLength - fieldX;
        }
        double err = Math.abs(fieldX - targetX);
        return err < Units.inchesToMeters(4);
    }
}
