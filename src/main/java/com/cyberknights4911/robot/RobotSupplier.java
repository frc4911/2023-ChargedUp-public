package com.cyberknights4911.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Supplier implementation for creating a new Robot.
 */
public final class RobotSupplier implements Supplier<RobotBase> {
    @Override
    public RobotBase get() {
        return new Robot();
    }
}
