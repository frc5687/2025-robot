package org.frc5687.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.vision.VisionSubsystem;
import org.frc5687.robot.util.TunableDouble;

public class DriveToGroundAlgae extends OutliersCommand {
    private final DriveSubsystem _drive;
    private final VisionSubsystem _vision;
    private final PIDController _distanceController;
    private final PIDController _yController;
    private final PIDController _thetaController;

    private static final double BLIND_DRIVE_TIME = 0.2;

    private static final TunableDouble distP = new TunableDouble("DriveToGroundAlgae", "distkP", 1.0);
    private static final TunableDouble yP = new TunableDouble("DriveToGroundAlgae", "ykP", 8.0);
    private static final TunableDouble angleP =
            new TunableDouble("DriveToGroundAlgae", "anglekP", 1.0);

    private static final TunableDouble horiz =
            new TunableDouble("DriveToGroundAlgae", "horiz offset", -0.1);

    private double lastDetected;

    public DriveToGroundAlgae(DriveSubsystem drive, VisionSubsystem vision) {
        _drive = drive;
        _vision = vision;
        _distanceController = new PIDController(distP.get(), 0.0, 0.0);
        _yController = new PIDController(yP.get(), 0.0, 0.0);
        _thetaController = new PIDController(angleP.get(), 0.0, 0.0);
        _thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(_drive);
    }

    @Override
    public void initialize() {
        _vision.setPipelineIndex("North_Camera", 1);
    }

    @Override
    protected void execute(double timestamp) {
        if (distP.hasChanged() || angleP.hasChanged() || yP.hasChanged()) {
            _distanceController.setP(distP.get());
            _yController.setP(yP.get());
            _thetaController.setP(angleP.get());
        }
        var detection = _vision.getClosestNeuralObservationOfType("North_Camera", 0);
        if (detection.isPresent()) lastDetected = Timer.getFPGATimestamp();

        if (Timer.getFPGATimestamp() - lastDetected > BLIND_DRIVE_TIME) {
            ChassisSpeeds speeds = new ChassisSpeeds();
            log("desired robot relative speeds", speeds, ChassisSpeeds.struct);
            _drive.setDesiredChassisSpeeds(speeds);
        } else if (detection.isPresent()) {
            double x = detection.get().getX();
            double y = detection.get().getY() + horiz.get();
            ChassisSpeeds speeds =
                    new ChassisSpeeds(
                            -_distanceController.calculate(Math.hypot(x, y)),
                            _yController.calculate(Math.atan2(y, x)),
                            _thetaController.calculate(Math.atan2(y, x)));
            log("desired robot relative speeds", speeds, ChassisSpeeds.struct);
            _drive.setDesiredChassisSpeeds(speeds);
        }
    }

    @Override
    public void end(boolean interrupted) {
        _vision.setPipelineIndex("North_Camera", 0);
    }
}
