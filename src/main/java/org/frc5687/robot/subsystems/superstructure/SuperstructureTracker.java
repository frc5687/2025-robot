package org.frc5687.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import org.frc5687.robot.RobotContainer;
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

    public boolean needsSafeCoralTransition() {
        // if (_container.getCoral().getArmAngleRads()
        //         < (CoralState.STOWED.getValue() - Units.degreesToRadians(10))) {
        //     System.out.println("Arm angle is to far in need to stow");
        //     return true;
        // }

        // if (_desiredState.getCoral() == CoralState.RECEIVE_FROM_FUNNEL) {
        //     System.out.println("Going to receive from funnel need to stow");
        //     return true;
        // }

        return false;
    }

    public boolean needToClearIntake() {
        if (_container.getIntake().getPivotArmAngleRads()
                < IntakeState.IDLE.getValue() + Units.degreesToRadians(10)) {
            return true;
        }
        return false;
    }
}
