package org.frc5687.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.subsystems.coralarm.CoralState;
import org.frc5687.robot.subsystems.intake.IntakeState;

public class SuperstructureTracker {
    private final RobotContainer _container;
    private SuperstructureState _desiredState;

    public SuperstructureTracker(RobotContainer container) {
        _container = container;
        _desiredState = SuperstructureGoals.SAFE_CORAL_TRANSITION;
    }

    // THIS IS A REQUIRED CALL BEFORE EVERY REQUEST TO CHANGE STATES
    public void setDesiredState(SuperstructureState state) {
        _desiredState = state;
    }

    public SuperstructureState getDesiredState() {
        return _desiredState;
    }

    public boolean needsSafeCoralTransition() {
        double currentArmAngle = _container.getCoral().getArmAngleRads();
        double stowedAngleThreshold = CoralState.STOWED.getArmAngle() + Units.degreesToRadians(10);

        // Check if arm is too far extended
        if (currentArmAngle > stowedAngleThreshold) {
            System.out.println(
                    "Arm angle is too far extended, need to stow: "
                            + Units.radiansToDegrees(currentArmAngle)
                            + " degrees");
            return true;
        }

        // Check if transitioning to receive from funnel
        if (_desiredState.getCoral() == CoralState.RECEIVE_FROM_FUNNEL) {
            System.out.println("Transitioning to receive from funnel, need to stow first");
            return true;
        }

        return false;
    }

    public boolean needToClearIntake() {
        if (_container.getIntake().getPivotArmAngleRads()
                > IntakeState.IDLE.getValue() + Units.degreesToRadians(10)) {
            // System.out.println("Need to clear intake");
            return true;
        }
        return false;
    }
}
