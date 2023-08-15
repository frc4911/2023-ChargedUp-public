package com.cyberknights4911.robot.model.quickdrop;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.cyberknights4911.robot.control.ButtonBinding;
import com.cyberknights4911.robot.control.DriveStickAction;
import com.cyberknights4911.robot.control.StickBinding;
import com.cyberknights4911.robot.control.Triggers;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class QuickDropControllerBinding implements ButtonBinding<QuickDropButtonAction>, StickBinding {
    private static final Trigger ALWAYS_FALSE = new Trigger(new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return false;
        }
    });
    private static final DoubleSupplier ALWAYS_ZERO = () -> 0.0;

    private final CommandXboxController driverController;

    public QuickDropControllerBinding() {
        this(new CommandXboxController(QuickDropPorts.Controller.DRIVER_CONTROLLER_PORT));
    }

    public QuickDropControllerBinding(CommandXboxController driverController) {
        this.driverController = driverController;
    }

    @Override
    public Triggers triggersFor(QuickDropButtonAction action) {
        switch(action) {
            case RESET_IMU: return new Triggers(driverController.y());
            case COLLECTOR_EXTEND: return new Triggers(driverController.rightTrigger());
            case COLLECTOR_RETRACT: return new Triggers( driverController.leftTrigger());
            case COLLECTOR_RUN_FORWARD: return new Triggers(driverController.rightBumper());
            case COLLECTOR_RUN_REVERSE: return new Triggers(driverController.leftBumper());
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
