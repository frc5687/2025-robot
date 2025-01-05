package org.frc5687.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.frc5687.robot.Constants;

public class ModuleIOSim implements ModuleIO {
    private final DCMotorSim _driveSim;
    private final DCMotorSim _turnSim;
    private final PIDController _driveController;
    private final PIDController _steerController;
    
    public ModuleIOSim() {
        var driveGearing = Constants.SwerveModule.GEAR_RATIO_DRIVE;
        var driveMotor = DCMotor.getKrakenX60Foc(1);
        var driveInertia = 0.025; // kg*m^2
        
        _driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                driveMotor,
                driveInertia, 
                driveGearing
            ),
            driveMotor
        );

        var turnGearing = Constants.SwerveModule.GEAR_RATIO_STEER;
        var turnMotor = DCMotor.getKrakenX60Foc(1);
        var turnInertia = 0.004; // kg*m^2

        _turnSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                turnMotor,
                turnInertia,
                turnGearing
            ),
            turnMotor
        );

        _driveController = new PIDController(0.5, 0, 0);
        _steerController = new PIDController(2.0, 0, 0);
        _steerController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        _driveSim.update(Constants.UPDATE_PERIOD);
        _turnSim.update(Constants.UPDATE_PERIOD);

        inputs.drivePositionRads = _driveSim.getAngularPositionRad();
        inputs.driveVelocityRadsPerSec = _driveSim.getAngularVelocityRadPerSec();
        inputs.turnAbsolutePosition = new Rotation2d(
            MathUtil.angleModulus(_turnSim.getAngularPositionRad())
        );
    }

    @Override
    public void runDriveVelocitySetpoint(double velocityRadsPerSec, double ffVolts) {
        _driveController.setSetpoint(velocityRadsPerSec);
        double voltage = MathUtil.clamp(_driveController.calculate(_driveSim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
        _driveSim.setInputVoltage(voltage);
    }

    @Override
    public void runTurnPositionSetpoint(double angleRads) {
        double currentAngle = _turnSim.getAngularPositionRad();
        double error = MathUtil.inputModulus(angleRads - currentAngle, -Math.PI, Math.PI);
        
        double pidOutput = Constants.SwerveModule.STEER_CONTROLLER_CONFIG.kP * error;
        double voltage = MathUtil.clamp(pidOutput, -12.0, 12.0);
        _turnSim.setInputVoltage(voltage);
    }
}