package com.cyberknights4911.robot;

import java.util.function.Supplier;

import com.cyberknights4911.robot.model.wham.Wham;

import edu.wpi.first.wpilibj.RobotBase;

public final class RobotSupplier implements Supplier<RobotBase> {

    @Override
    public RobotBase get() {
        return new Robot(this::createWham);
    }

    private RobotStateListener createWham() {
        return new Wham();
    }
}
