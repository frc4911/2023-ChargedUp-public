package com.cyberknights4911.robot.model.quickdrop;

import java.util.function.DoubleSupplier;
import com.cyberknights4911.robot.control.ButtonBinding;
import com.cyberknights4911.robot.control.DriveStickAction;
import com.cyberknights4911.robot.control.StickBinding;
import com.cyberknights4911.robot.control.Triggers;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class QuickDropControllerBinding implements ButtonBinding<QuickDropButtonAction>, StickBinding {
    private final LogitechController driverController;

    public QuickDropControllerBinding() {
        this.driverController = new LogitechController(QuickDropPorts.Controller.DRIVER_CONTROLLER_PORT);
    }

    @Override
    public Triggers triggersFor(QuickDropButtonAction action) {
        switch(action) {
            // case RESET_IMU: return new Triggers(driverController.y());
            // case COLLECTOR_EXTEND: return new Triggers(driverController.rightTrigger());
            // case COLLECTOR_RETRACT: return new Triggers( driverController.leftTrigger());
            case COLLECTOR_RUN_FORWARD: return new Triggers(driverController.rightBumper());
            case COLLECTOR_RUN_REVERSE: return new Triggers(driverController.leftBumper());
            // case INDEXER_RUN: return new Triggers(driverController.a());
            // case SHOOTER_BLAST: return new Triggers(driverController.b());
            case HOOD_POSITION_0: return new Triggers(driverController.povDown());
            case HOOD_POSITION_25: return new Triggers(driverController.povLeft());
            case HOOD_POSITION_50: return new Triggers(driverController.povUp());
            case HOOD_POSITION_100: return new Triggers(driverController.povRight());
            default: return new Triggers(ALWAYS_FALSE);
        }
    }

    @Override
    public DoubleSupplier supplierFor(DriveStickAction action) {
        switch (action) {
            case FORWARD: return driverController::getLeftY;
            case STRAFE: return driverController::getLeftX;
            case ROTATE: return driverController::getRightX;
            default: return ALWAYS_ZERO;
        }
    }
}
