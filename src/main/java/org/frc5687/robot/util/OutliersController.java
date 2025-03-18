package org.frc5687.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OutliersController {
    private CommandXboxController _xboxController;
    private CommandPS4Controller _ps4Controller;
    private CommandPS5Controller _ps5Controller;
    private CommandGenericHID _genericController;

    public OutliersController(CommandGenericHID controller) {
        _genericController = controller;
        if (controller instanceof CommandXboxController) {
            _xboxController = (CommandXboxController) controller;
        } else if (controller instanceof CommandPS4Controller) {
            _ps4Controller = (CommandPS4Controller) controller;
        } else if (controller instanceof CommandPS5Controller) {
            _ps5Controller = (CommandPS5Controller) controller;
        } else {
            System.out.println("ERORR EORORROR ERORROR THE CONTROLLER IS NOT A SUPPORTED TYPE");
        }
    }

    public Trigger a() {
        if (_xboxController != null) {
            return _xboxController.a();
        } else if (_ps4Controller != null) {
            return _ps4Controller.cross();
        } else {
            return _ps5Controller.cross();
        }
    }

    public Trigger b() {
        if (_xboxController != null) {
            return _xboxController.b();
        } else if (_ps4Controller != null) {
            return _ps4Controller.circle();
        } else {
            return _ps5Controller.circle();
        }
    }

    public Trigger x() {
        if (_xboxController != null) {
            return _xboxController.x();
        } else if (_ps4Controller != null) {
            return _ps4Controller.square();
        } else {
            return _ps5Controller.square();
        }
    }

    public Trigger y() {
        if (_xboxController != null) {
            return _xboxController.y();
        } else if (_ps4Controller != null) {
            return _ps4Controller.triangle();
        } else {
            return _ps5Controller.triangle();
        }
    }

    public Trigger leftBumper() {
        if (_xboxController != null) {
            return _xboxController.leftBumper();
        } else if (_ps4Controller != null) {
            return _ps4Controller.L1();
        } else {
            return _ps5Controller.L1();
        }
    }

    public Trigger rightBumper() {
        if (_xboxController != null) {
            return _xboxController.rightBumper();
        } else if (_ps4Controller != null) {
            return _ps4Controller.R1();
        } else {
            return _ps5Controller.R1();
        }
    }

    public Trigger leftTrigger() {
        if (_xboxController != null) {
            return _xboxController.leftTrigger();
        } else if (_ps4Controller != null) {
            return _ps4Controller.L2();
        } else {
            return _ps5Controller.L2();
        }
    }

    public Trigger rightTrigger() {
        if (_xboxController != null) {
            return _xboxController.rightTrigger();
        } else if (_ps4Controller != null) {
            return _ps4Controller.R2();
        } else {
            return _ps5Controller.R2();
        }
    }

    public Trigger start() {
        if (_xboxController != null) {
            return _xboxController.start();
        } else if (_ps4Controller != null) {
            return _ps4Controller.options();
        } else {
            return _ps5Controller.options();
        }
    }

    public Trigger back() {
        if (_xboxController != null) {
            return _xboxController.back();
        } else if (_ps4Controller != null) {
            return _ps4Controller.share();
        } else {
            return _ps5Controller.create();
        }
    }

    public Trigger povUp() {
        return _genericController.povUp();
    }

    public Trigger povUpRight() {
        return _genericController.povUpRight();
    }

    public Trigger povRight() {
        return _genericController.povRight();
    }

    public Trigger povDownRight() {
        return _genericController.povDownRight();
    }

    public Trigger povDown() {
        return _genericController.povDown();
    }

    public Trigger povDownLeft() {
        return _genericController.povDownLeft();
    }

    public Trigger povLeft() {
        return _genericController.povLeft();
    }

    public Trigger povUpLeft() {
        return _genericController.povUpLeft();
    }

    public Trigger rightMiddleButton() {
        if (_xboxController != null) {
            return _xboxController.start();
        } else if (_ps4Controller != null) {
            return _ps4Controller.options();
        } else {
            return _ps5Controller.options();
        }
    }

    public Trigger leftMiddleButton() {
        if (_xboxController != null) {
            return _xboxController.back();
        } else if (_ps4Controller != null) {
            return _ps4Controller.share();
        } else {
            return _ps5Controller.create();
        }
    }

    public double getLeftX() {
        if (_xboxController != null) {
            return _xboxController.getLeftX();
        } else if (_ps4Controller != null) {
            return _ps4Controller.getLeftX();
        } else {
            return _ps5Controller.getLeftX();
        }
    }

    public double getLeftY() {
        if (_xboxController != null) {
            return _xboxController.getLeftY();
        } else if (_ps4Controller != null) {
            return _ps4Controller.getLeftY();
        } else {
            return _ps5Controller.getLeftY();
        }
    }

    public double getRightX() {
        if (_xboxController != null) {
            return _xboxController.getRightX();
        } else if (_ps4Controller != null) {
            return _ps4Controller.getRightX();
        } else {
            return _ps5Controller.getRightX();
        }
    }

    public double getRightY() {
        if (_xboxController != null) {
            return _xboxController.getRightY();
        } else if (_ps4Controller != null) {
            return _ps4Controller.getRightY();
        } else {
            return _ps5Controller.getRightY();
        }
    }

    public Trigger leftJoystickButton() {
        if (_xboxController != null) {
            return _xboxController.leftStick();
        } else if (_ps4Controller != null) {
            return _ps4Controller.L3();
        } else {
            return _ps5Controller.L3();
        }
    }

    public Trigger rightJoystickButton() {
        if (_xboxController != null) {
            return _xboxController.rightStick();
        } else if (_ps4Controller != null) {
            return _ps4Controller.R3();
        } else {
            return _ps5Controller.R3();
        }
    }
}
