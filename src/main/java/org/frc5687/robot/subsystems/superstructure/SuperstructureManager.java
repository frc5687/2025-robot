package org.frc5687.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import java.util.Optional;
import java.util.function.Supplier;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.algae.IntakeAlgae;
import org.frc5687.robot.subsystems.algaearm.AlgaeState;
import org.frc5687.robot.util.EpilogueLog;
import org.frc5687.robot.util.FieldConstants;

public class SuperstructureManager extends SubsystemBase implements EpilogueLog {
    public enum SuperstructureMode {
        CORAL,
        ALGAE
    }

    private final RobotContainer _container;
    private final RequestHandler _requestHandler;
    private boolean _forceQueueExecution = false;

    private SuperstructureMode _currentMode = SuperstructureMode.CORAL;
    private Optional<Pose2d> _goalPose = Optional.empty();

    public SuperstructureManager(RobotContainer container) {
        _container = container;
        _requestHandler = new RequestHandler(container, this::isCoralMode);
    }

    public SuperstructureMode getCurrentMode() {
        return _currentMode;
    }

    public void toggleMode() {
        if (_currentMode == SuperstructureMode.ALGAE) {
            _currentMode = SuperstructureMode.CORAL;
        } else {
            _currentMode = SuperstructureMode.ALGAE;
        }
    }

    public boolean isCoralMode() {
        return _currentMode == SuperstructureMode.CORAL;
    }

    public boolean isAlgaeMode() {
        return _currentMode == SuperstructureMode.ALGAE;
    }

    /**
     * This creates a request to the superstructure for a desired state while having the option to
     * queue.
     *
     * @param stateSupplier A supplier for the desired superstructure state positions
     * @param description debugging description of the action
     * @param type RequestType to queue or not to queue
     * @return Command for execution
     */
    public Command createRequest(
            Supplier<SuperstructureState> stateSupplier, String description, RequestType type) {
        return new FunctionalCommand(
                // init
                () ->
                        _requestHandler.handleNewRequest(
                                new SuperstructureRequest(
                                        stateSupplier.get(),
                                        type,
                                        () ->
                                                type != RequestType.QUEUED
                                                        || _forceQueueExecution
                                                        || isRobotWithinGoalPose()
                                                        || isElevatorGoingDown(stateSupplier.get()),
                                        description)),
                // execute
                () -> {},
                // end
                (interrupted) -> {
                    if (interrupted) {
                        System.out.println("Request interrupted: " + description);
                    }
                },
                // IsFinished
                () -> {
                    SuperstructureRequest activeRequest = _requestHandler.getActiveRequest();
                    return activeRequest == null;
                },
                this);
    }

    public Command createRequest(SuperstructureState placeState, RequestType type) {
        return createRequest(() -> placeState, "Set place height " + placeState.getElevator(), type);
    }

    public Command receiveFunnel(RequestType type) {
        // receiving from funnel, explicitly switch to CORAL mode
        return new SequentialCommandGroup(
                // new InstantCommand(() -> setMode(SuperstructureMode.CORAL)),
                createRequest(Constants.SuperstructureGoals.RECEIVE_FROM_FUNNEL, type), indexCoral());
    }

    public Command indexCoral() {
        return new FunctionalCommand(
                () -> {
                    _container.getCoral().setWheelMotorDutyCycle(0.3);
                },
                () -> {},
                (interrupted) -> {
                    double currentPos = _container.getCoral().getWheelMotorPosition();
                    _container.getCoral().setWheelMotorPosition(currentPos + 1.5);
                },
                _container.getCoral()::isCoralDetected,
                _container.getCoral());
    }

    public Command receiveFunnelSim(RequestType type) {
        // receiving from funnel, explicitly switch to CORAL mode
        return new SequentialCommandGroup(
                // new InstantCommand(() -> setMode(SuperstructureMode.CORAL)),
                createRequest(Constants.SuperstructureGoals.RECEIVE_FROM_FUNNEL, type));
    }

    public Command grabAlgae(SuperstructureState state, RequestType type) {
        return new SequentialCommandGroup(
                // new InstantCommand(() -> setMode(SuperstructureMode.ALGAE)),
                createRequest(state, type).until(() -> _container.getAlgae().isAlgaeDetected()));
    }

    public Command algaeIntake(SuperstructureState state) {
        return new SequentialCommandGroup(
                        grabAlgae(state, RequestType.IMMEDIATE),
                        new IntakeAlgae(_container.getAlgae()),
                        new WaitUntilCommand(
                                () ->
                                        _container
                                                        .getDrive()
                                                        .getPose()
                                                        .getTranslation()
                                                        .getDistance(FieldConstants.getAllianceSpecificReefCenter())
                                                > 2),
                        createRequest(
                                new SuperstructureState(
                                        Optional.empty(),
                                        Optional.empty(),
                                        Optional.of(AlgaeState.IDLE),
                                        Optional.empty()),
                                RequestType.IMMEDIATE),
                        new InstantCommand(() -> _container.getAlgae().setWheelMotorVoltage(0)))
                .withName("Algae Reef Intake");
    }

    public Command aimAtAlgaeNet() {
        // return new SequentialCommandGroup(
        //         // new InstantCommand(() -> setMode(SuperstructureMode.ALGAE)),
        return createRequest(Constants.SuperstructureGoals.BARGE_DROPOFF, RequestType.IMMEDIATE);
        // new InstantCommand(()->_container.getAlgae().setWheelMotorVoltage(-12)));
    }

    // Queue override controls
    public void forceQueueExecution() {
        _forceQueueExecution = true;
    }

    public void releaseQueueExecution() {
        _forceQueueExecution = false;
    }

    private boolean isPositionNearReef() {
        Translation2d currentPose = _container.getDrive().getPose().getTranslation();
        Translation2d reefCenter = FieldConstants.getAllianceSpecificReefCenter();
        return currentPose.getDistance(reefCenter) <= 2.0;
    }

    public void setGoalPose(Optional<Pose2d> goalPose) {
        _goalPose = goalPose;
    }

    private boolean isRobotWithinGoalPose() {
        if (_goalPose.isEmpty()) {
            return false;
        }

        Translation2d allianceReefCenter = FieldConstants.getAllianceSpecificReefCenter();
        Translation2d currentPose = _container.getDrive().getPose().getTranslation();
        Translation2d goalPose = _goalPose.get().getTranslation();

        double distanceFromCenterToRobot = currentPose.getDistance(allianceReefCenter);
        double distanceFromCenterToGoal = goalPose.getDistance(allianceReefCenter);

        // Return true if either:
        // 1. Robot is at the goal pose (within a small threshold)
        // 2. Robot is between the center and goal (robot is closer to center than goal is)
        double atGoalThreshold = 0.2;

        return currentPose.getDistance(goalPose) < atGoalThreshold
                || (distanceFromCenterToRobot <= distanceFromCenterToGoal
                        && isBetweenCenterAndGoal(currentPose, goalPose, allianceReefCenter));
    }

    private boolean isBetweenCenterAndGoal(
            Translation2d robotPose, Translation2d goalPose, Translation2d centerPose) {
        Translation2d centerToGoal = goalPose.minus(centerPose);
        Translation2d centerToRobot = robotPose.minus(centerPose);

        return centerToGoal.getX() * centerToRobot.getX() + centerToGoal.getY() * centerToRobot.getY()
                > 0;
    }

    private boolean isElevatorGoingDown(SuperstructureState requestedState) {
        if (requestedState.getElevator().isEmpty()) return false; // FIXME is this the correct behavior?

        double elevatorHeight = _container.getElevator().getElevatorHeight();
        return requestedState.getElevator().get().getHeight() < elevatorHeight;
    }

    @Override
    public void periodic() {
        _requestHandler.execute();
        if (_goalPose.isPresent()) {
            log("Current Goal Pose", _goalPose.get(), Pose2d.struct);
        }
    }

    @Override
    public String getLogBase() {
        return "SuperstructureManager";
    }
}
