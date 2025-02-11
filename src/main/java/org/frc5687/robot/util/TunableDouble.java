package org.frc5687.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;

public class TunableDouble {
    private final String _key;
    private final double _defaultValue;
    private final NetworkTableEntry _entry;
    private double _value;
    private double _lastValue;
    private boolean _hasChanged;

    public TunableDouble(String table, String key, double defaultValue) {
        _key = table + "/" + key;
        _defaultValue = defaultValue;

        _value = Preferences.getDouble(_key, defaultValue);
        _lastValue = _value;

        NetworkTable networkTable = NetworkTableInstance.getDefault().getTable(table);
        _entry = networkTable.getEntry(key);
        _entry.setDouble(_value);
        _hasChanged = false;
    }

    public double get() {
        _lastValue = _value;
        double networkValue = _entry.getDouble(_value);
        if (networkValue != _value) {
            _value = networkValue;
            _hasChanged = true;
            Preferences.setDouble(_key, _value);
        }
        return _value;
    }

    public boolean hasChanged() {
        get();
        boolean changed = _hasChanged;
        _hasChanged = false;
        return changed;
    }

    public double getLastValue() {
        return _lastValue;
    }

    public void set(double value) {
        if (_value != value) {
            _lastValue = _value;
            _value = value;
            _hasChanged = true;
            _entry.setDouble(value);
            Preferences.setDouble(_key, value);
        }
    }

    public void reset() {
        set(_defaultValue);
    }

    public String getKey() {
        return _key;
    }

    public double getDefault() {
        return _defaultValue;
    }
}
