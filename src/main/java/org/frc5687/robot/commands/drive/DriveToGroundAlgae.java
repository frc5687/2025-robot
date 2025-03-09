package org.frc5687.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.vision.VisionSubsystem;

public class DriveToGroundAlgae extends OutliersCommand {
    private final DriveSubsystem _drive;
    private final VisionSubsystem _vision;
    private final PIDController _distanceController;
    private final PIDController _thetaController;

    private static final double BLIND_DRIVE_TIME = 0.2;

    private double lastDetected;

    public DriveToGroundAlgae(DriveSubsystem drive, VisionSubsystem vision) {
        _drive = drive;
        _vision = vision;
        _distanceController = new PIDController(3.0, 0.0, 0.2);
        _thetaController = new PIDController(4.0, 0.0, 0.0);
        _thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        _vision.setPipelineIndex("North_Camera", 1);
    }

    @Override
    protected void execute(double timestamp) {
        var detection = _vision.getClosestNeuralObservationOfType("North_Camera", 0);
        if (detection.isPresent()) lastDetected = Timer.getFPGATimestamp();

        ChassisSpeeds speeds;
        if (Timer.getFPGATimestamp() - lastDetected > BLIND_DRIVE_TIME) {
            speeds = new ChassisSpeeds();
        } else {
            speeds =
                    new ChassisSpeeds(
                            _distanceController.calculate(detection.get().getDistance()),
                            0.0,
                            _thetaController.calculate(detection.get().getAngle()));
        }
        log("desired robot relative speeds", speeds, ChassisSpeeds.struct);
        _drive.setDesiredChassisSpeeds(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        _vision.setPipelineIndex("North_Camera", 0);
    }
}
