package com.cyberknights4911.robot.constants;

import edu.wpi.first.wpilibj.Preferences;

public final class BooleanPreference {
    private final String key;
    private final boolean defaultValue;

    public BooleanPreference(String key, boolean defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
    }

    public boolean getValue() {
        return Preferences.getBoolean(key, defaultValue);
    }
}
