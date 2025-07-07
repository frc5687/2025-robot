package org.frc5687.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import java.util.Optional;
import java.util.function.Supplier;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.algae.IntakeAlgae;
import org.frc5687.robot.commands.coral.IntakeAndIndexCoral;
import org.frc5687.robot.commands.intake.GroundIndexCoral;
import org.frc5687.robot.commands.intake.IntakeFromGround;
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
        _requestHandler = new RequestHandler(container);
    }

    public SuperstructureMode getCurrentMode() {
        return _currentMode;
    }

    public void toggleMode() {
        if (_currentMode == SuperstructureMode.ALGAE) {
            setCoralMode();
        } else {
            setAlgaeMode();
        }
    }

    public void setCoralMode() {
        _currentMode = SuperstructureMode.CORAL;
    }

    public void setAlgaeMode() {
        _currentMode = SuperstructureMode.ALGAE;
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
                                        () -> {
                                            if (type == RequestType.QUEUED) {
                                                return _forceQueueExecution
                                                        || (isRobotWithinGoalPose() && canElevatorGoUp(stateSupplier.get()))
                                                        || isElevatorGoingDown(stateSupplier.get());
                                            } else if (type == RequestType.AUTO_SEQUENCE) {
                                                return canElevatorGoUpAuto(stateSupplier.get())
                                                        || isElevatorGoingDown(stateSupplier.get());
                                            } else return true;
                                        },
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
                });
    }

    public Command createRequest(SuperstructureState placeState, RequestType type) {
        return createRequest(() -> placeState, "Set place height " + placeState.getElevator(), type);
    }

    public RequestHandler getRequestHandler() {
        return _requestHandler;
    }

    public Command runIntake() {
        return new InstantCommand(
                () -> {
                    new IntakeFromGround(_container.getIntake());
                });
    }

    public Command receiveFunnel(RequestType type) {
        SuperstructureRequest currentRequest =
                new SuperstructureRequest(
                        Constants.SuperstructureGoals.RECEIVE_FROM_FUNNEL, type, () -> true, "req");
        return createRequest(Constants.SuperstructureGoals.RECEIVE_FROM_FUNNEL, type)
                // .andThen(
                //         new InstantCommand(
                //                 () -> {
                //                     SuperstructureRequest currentRequest =
                // _requestHandler.getActiveRequest();
                //                     if (currentRequest == null) {
                //                         currentRequest =
                //                                 new SuperstructureRequest(
                //
                // Constants.SuperstructureGoals.RECEIVE_FROM_FUNNEL,
                //                                         type,
                //                                         () -> true,
                //                                         "req");
                //                     }
                //                     // This only has worked if a schedule, I'm not sure why???
                //                     new IntakeAndIndexCoral(_container.getCoral(), this,
                // currentRequest).schedule();
                //                 }));
                .andThen(new IntakeAndIndexCoral(_container.getCoral(), this, currentRequest));
    }

    public Command receiveFromGroundIntake(RequestType type) {
        SuperstructureRequest currentRequest =
                new SuperstructureRequest(
                        Constants.SuperstructureGoals.RECEIVE_FROM_GROUND_INTAKE, type, () -> true, "req");
        return createRequest(Constants.SuperstructureGoals.RECEIVE_FROM_GROUND_INTAKE, type)
                .andThen(
                        new GroundIndexCoral(
                                _container.getCoral(), _container.getIntake(), this, currentRequest));
    }

    public Command autoReceiveFunnel() {
        return new SequentialCommandGroup(
                createRequest(Constants.SuperstructureGoals.RECEIVE_FROM_FUNNEL, RequestType.IMMEDIATE),
                new IntakeAndIndexCoral(_container.getCoral(), this, _requestHandler.getActiveRequest()));
    }

    public Command receiveFunnelSim(RequestType type) {
        return createRequest(Constants.SuperstructureGoals.RECEIVE_FROM_FUNNEL, type);
    }

    public Command grabAlgae(SuperstructureState state, RequestType type) {
        return createRequest(state, type).until(() -> _container.getAlgae().isAlgaeDetected());
    }

    public Command algaeIntake(SuperstructureState state) {
        return new SequentialCommandGroup(
                        createRequest(state, RequestType.IMMEDIATE),
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

    public Command algaeIntakeAuto() {
        return new SequentialCommandGroup(
                        createRequest(
                                new SuperstructureState(
                                        Optional.empty(),
                                        Optional.empty(),
                                        Optional.of(AlgaeState.REEF_PICKUP),
                                        Optional.empty()),
                                RequestType.IMMEDIATE),
                        new IntakeAlgae(_container.getAlgae()))
                .withName("AlgaeIntakeAuto");
    }

    public Command algaeGroundIntakeAuto() {
        return new SequentialCommandGroup(
                        createRequest(
                                new SuperstructureState(
                                        Optional.empty(),
                                        Optional.empty(),
                                        Optional.of(AlgaeState.GROUND_PICKUP),
                                        Optional.empty()),
                                RequestType.IMMEDIATE),
                        new IntakeAlgae(_container.getAlgae()))
                .withName("AlgaeGroundIntakeAuto");
    }

    public Command hybridAlgaeIntake() {

        return new SequentialCommandGroup(
                        createRequest(
                                new SuperstructureState(
                                        Optional.empty(),
                                        Optional.empty(),
                                        Optional.of(AlgaeState.REEF_PICKUP),
                                        Optional.empty()),
                                RequestType.IMMEDIATE),
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
        return createRequest(Constants.SuperstructureGoals.BARGE_DROPOFF, RequestType.IMMEDIATE);
    }

    public void forceQueueExecution() {
        _forceQueueExecution = true;
    }

    public void releaseQueueExecution() {
        _forceQueueExecution = false;
    }

    public void setGoalPose(Optional<Pose2d> goalPose) {
        _goalPose = goalPose;
    }

    // private SuperstructureState getalgaeheight() {
    //     SuperstructureState targetState = Constants.SuperstructureGoals.RECEIVE_FROM_FUNNEL;
    //     int currentFace = 1;
    //     Supplier<Pose2d> rawPose = () -> _container.getDrive().getPose();
    //     var alliance = DriverStation.getAlliance();
    //     if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
    //         Pose2d mirroredPose = FlippingUtil.flipFieldPose(rawPose.get());
    //         currentFace = ReefAlignmentHelpers.calculateBestFace(mirroredPose);
    //         return ReefAlignmentHelpers.calculateAlgaeHeight(currentFace);
    //     } else {
    //         currentFace = ReefAlignmentHelpers.calculateBestFace(rawPose.get());

    //         return ReefAlignmentHelpers.calculateAlgaeHeight(currentFace);
    //     }
    // }

    private boolean isRobotWithinGoalPose() {
        if (_goalPose.isEmpty()) {
            return false;
        }

        Translation2d allianceReefCenter = FieldConstants.getAllianceSpecificReefCenter();
        Translation2d currentPose = _container.getDrive().getPose().getTranslation();
        Translation2d goalPose = _goalPose.get().getTranslation();

        double distanceFromCenterToRobot = currentPose.getDistance(allianceReefCenter);
        double distanceFromCenterToGoal = goalPose.getDistance(allianceReefCenter);

        double atGoalThreshold = 0.3;

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

    private boolean canElevatorGoUp(SuperstructureState requestedState) {
        if (requestedState.getElevator().isEmpty()) return false;

        double currentElevatorHeight = _container.getElevator().getElevatorHeight();
        double requestedHeight = requestedState.getElevator().get().getHeight();
        boolean isMovingUp = requestedHeight > currentElevatorHeight;

        if (!isMovingUp) return false;

        boolean intakeInReceivePosition =
                _container
                        .getIntake()
                        .isAtState(Constants.SuperstructureGoals.RECEIVE_FROM_GROUND_INTAKE.getIntake().get());

        if (intakeInReceivePosition) {
            System.out.println("Cannot move elevator - intake is in receive position");
            return false;
        }

        if (isCoralMode()) {
            return _container.getCoral().isCoralDetected();
        }

        return true;
    }

    // private boolean canElevatorGoUp(SuperstructureState requestedState) {
    //     if (requestedState.getElevator().isEmpty()) return false;

    //     double currentElevatorHeight = _container.getElevator().getElevatorHeight();
    //     double requestedHeight = requestedState.getElevator().get().getHeight();
    //     boolean isMovingUp = requestedHeight > currentElevatorHeight;

    //     if (!isMovingUp) return false;

    //     if (isCoralMode()) {
    //         return _container.getCoral().isCoralDetected();
    //     }

    //     return true;
    // }

    private boolean canElevatorGoUpAuto(SuperstructureState requestedState) {
        // // Check if intake is in receive-from-intake position
        // boolean intakeInReceivePosition =
        //         _container
        //                 .getIntake()
        //
        // .isAtState(Constants.SuperstructureGoals.RECEIVE_FROM_GROUND_INTAKE.getIntake().get());

        // if (intakeInReceivePosition) {
        //     System.out.println("Cannot move elevator in auto - intake is in receive position");
        //     return false;
        // }

        return _container.getCoral().isCoralDetected();
    }

    // private boolean canElevatorGoUpAuto(SuperstructureState requestedState) {
    //     // if (requestedState.getElevator().isEmpty()) return false;

    //     // double currentElevatorHeight = _container.getElevator().getElevatorHeight();
    //     // double requestedHeight = requestedState.getElevator().get().getHeight();
    //     // boolean isMovingUp = requestedHeight > currentElevatorHeight;

    //     // if (!isMovingUp) return false;
    //     System.out.println(_container.getCoral().isCoralDetected());
    //     return _container.getCoral().isCoralDetected();
    // }

    private boolean isElevatorGoingDown(SuperstructureState requestedState) {
        if (requestedState.getElevator().isEmpty()) return false;

        double elevatorHeight = _container.getElevator().getElevatorHeight();
        boolean isMovingDown = requestedState.getElevator().get().getHeight() < elevatorHeight;

        if (!isMovingDown) return false;

        // Check if intake is in receive-from-intake position
        boolean intakeInReceivePosition =
                _container
                        .getIntake()
                        .isAtState(Constants.SuperstructureGoals.RECEIVE_FROM_GROUND_INTAKE.getIntake().get());

        if (intakeInReceivePosition) {
            System.out.println("Cannot move elevator - intake is in receive position");
            return false;
        }

        return true;
    }

    // private boolean isElevatorGoingDown(SuperstructureState requestedState) {
    //     if (requestedState.getElevator().isEmpty()) return false; // FIXME is this the correct
    //     // behavior?

    //     double elevatorHeight = _container.getElevator().getElevatorHeight();
    //     return requestedState.getElevator().get().getHeight() < elevatorHeight;
    // }

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
