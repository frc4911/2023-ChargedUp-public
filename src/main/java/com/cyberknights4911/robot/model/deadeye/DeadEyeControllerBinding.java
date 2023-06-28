package com.cyberknights4911.robot.model.deadeye;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.control.ControllerBinding;
import com.cyberknights4911.robot.control.DriveStickAction;
import com.cyberknights4911.robot.control.Triggers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class DeadEyeControllerBinding implements ControllerBinding<DeadEyeButtonAction> {
    private static final Trigger ALWAYS_FALSE = new Trigger(new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return false;
        }
    });
    private static final DoubleSupplier ALWAYS_ZERO = () -> 0.0;

    private final CommandXboxController driverController;

    public DeadEyeControllerBinding() {
        this(new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT));
    }

    public DeadEyeControllerBinding(CommandXboxController driverController) {
        this.driverController = driverController;
    }

    @Override
    public Triggers triggersFor(DeadEyeButtonAction action) {
        switch(action) {
            case RESET_IMU: return new Triggers(driverController.y());
            default: return new Triggers(ALWAYS_FALSE);
        }
    }

    @Override
    public DoubleSupplier supplierFor(DriveStickAction action) {
        switch (action) {
            case FORWARD: return this::getForwardInput;
            case STRAFE: return this::getStrafeInput;
            case ROTATE: return this::getRotationInput;
            default: return ALWAYS_ZERO;
        }
    }

    private static double deadband(double value, double tolerance) {
      if (Math.abs(value) < tolerance) {
        return 0.0;
      }
  
      return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }
  
    private static double square(double value) {
      return Math.copySign(value * value, value);
    }
  
    private double getForwardInput() {
        return -square(deadband(driverController.getLeftY(), Constants.DRIVE_DEADBAND));
    }
  
    private double getStrafeInput() {
        return -square(deadband(driverController.getLeftX(), Constants.DRIVE_DEADBAND));
    }
  
    private double getRotationInput() {
        return -square(deadband(driverController.getRightX(), Constants.DRIVE_DEADBAND));
    }
    
}
