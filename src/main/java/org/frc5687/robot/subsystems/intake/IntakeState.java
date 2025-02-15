package org.frc5687.robot.subsystems.intake;

public enum IntakeState {
    DEPLOYED(0, 0, 0), // 180 deg
    STOWED(Math.PI / 4.0, 0, 0),
    PASSOFF_TO_CORAL(2.22, -12, 12), // 130 deg
    IDLE(Math.PI / 2.0, 0, 0),
    EJECT(0, 12, -12); // 90 deg

    private final double _value;
    private final double _beltVoltage;
    private final double _rollerVoltage;

    IntakeState(double value, double beltVoltage, double rollerVoltage) {
        _value = value;
        _beltVoltage = beltVoltage;
        _rollerVoltage = rollerVoltage;
    }

    public double getValue() {
        return _value;
    }

    public double getBeltVoltage() {
        return _beltVoltage;
    }

    public double getRollerVoltage() {
        return _rollerVoltage;
    }
}
