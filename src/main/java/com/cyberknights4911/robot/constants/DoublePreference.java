package com.cyberknights4911.robot.constants;

import edu.wpi.first.wpilibj.Preferences;

/** Convenience class for defining double constants that can be modified via ShuffleBoard. */
public final class DoublePreference {
    private final String key;
    private final double defaultValue;

    public DoublePreference(String key, double defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
    }

    public double getValue() {
        return Preferences.getDouble(key, defaultValue);
    }
}
