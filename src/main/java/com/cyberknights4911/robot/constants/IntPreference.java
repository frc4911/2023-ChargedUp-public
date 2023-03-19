package com.cyberknights4911.robot.constants;

import edu.wpi.first.wpilibj.Preferences;

/** Convenience class for defining integer constants that can be modified via ShuffleBoard. */
public final class IntPreference {
    private final String key;
    private final int defaultValue;

    public IntPreference(String key, int defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
    }

    public int getValue() {
        return Preferences.getInt(key, defaultValue);
    } 
}
