package com.cyberknights4911.robot.model.deadeye;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.cyberknights4911.robot.control.ButtonBinding;
import com.cyberknights4911.robot.control.DriveStickAction;
import com.cyberknights4911.robot.control.StickBinding;
import com.cyberknights4911.robot.control.Triggers;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class DeadEyeControllerBinding implements ButtonBinding<DeadEyeButtonAction>, StickBinding {
    private static final Trigger ALWAYS_FALSE = new Trigger(new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return false;
        }
    });
    private static final DoubleSupplier ALWAYS_ZERO = () -> 0.0;

    private final CommandXboxController driverController;

    public DeadEyeControllerBinding() {
        this(new CommandXboxController(DeadEyePorts.Controller.DRIVER_CONTROLLER_PORT));
    }

    public DeadEyeControllerBinding(CommandXboxController driverController) {
        this.driverController = driverController;
    }

    @Override
    public Triggers triggersFor(DeadEyeButtonAction action) {
        if (Objects.requireNonNull(action) == DeadEyeButtonAction.RESET_IMU) {
            return new Triggers(driverController.y());
        }
        return new Triggers(ALWAYS_FALSE);
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
