package com.cyberknights4911.robot;

import java.util.function.Supplier;

import com.cyberknights4911.robot.model.deadeye.DeadEye;
import com.cyberknights4911.robot.model.quickdrop.QuickDrop;
import com.cyberknights4911.robot.model.wham.Wham;

import edu.wpi.first.wpilibj.RobotBase;

public final class RobotSupplier implements Supplier<RobotBase> {

    @Override
    public RobotBase get() {
        // Change the passed arg to change the robot.
        return new Robot(this::createQuickDrop);
    }

    private RobotStateListener createWham() {
        return new Wham();
    }

    private RobotStateListener createQuickDrop() {
        return new QuickDrop();
    }

    private RobotStateListener createDeadEye() {
        return new DeadEye();
    }
}
