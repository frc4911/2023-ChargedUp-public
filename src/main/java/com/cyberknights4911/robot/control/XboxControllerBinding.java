package com.cyberknights4911.robot.control;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.cyberknights4911.robot.constants.Constants;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Default controller binding. Do not place alternate bindings in this class. Instead, create a new
 * ControllerBinding specific to the alternate control scheme and swap the binding out in
 * RobotContainer.
 */
public final class XboxControllerBinding implements ControllerBinding {
    private static final Trigger ALWAYS_FALSE = new Trigger(new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return false;
        }
    });
    private static final DoubleSupplier ALWAYS_ZERO = () -> 0.0;

    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    public XboxControllerBinding() {
        this(
            new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT),
            new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT)
        );
    }

    public XboxControllerBinding(
        CommandXboxController driverController, CommandXboxController operatorController
    ) {
        this.driverController = driverController;
        this.operatorController = operatorController;
    }

    @Override
    public List<Trigger> triggerFor(ButtonAction action) {
        switch(action) {
            case ALIGN_COLLECT: return Arrays.asList(driverController.a());
            case RESET_IMU: return Arrays.asList(driverController.y());
            case COLLECTOR_BACKWARD: return Arrays.asList(driverController.rightBumper(), driverController.leftBumper());
            case COLLECTOR_FORWARD: return Arrays.asList(driverController.rightTrigger(), driverController.leftTrigger());
            case CLIMB_WHEEL_LOCK: return Arrays.asList(driverController.povDown());
            case RESET_WHEELS: return Arrays.asList(driverController.start());
            case STOW: return Arrays.asList(operatorController.a());
            case ARM_L3: return Arrays.asList(operatorController.b());
            case ARM_L2: return Arrays.asList(operatorController.x());
            case CLIMB_DEPLOY: return Arrays.asList(operatorController.y());
            case CLIMB_LOCKOUT: return Arrays.asList(operatorController.rightBumper());
            case BOB_STOW: return Arrays.asList(operatorController.rightTrigger());
            case BOB_DEPLOY: return Arrays.asList(operatorController.leftTrigger());
            case REAR_COLLECT: return Arrays.asList(operatorController.povUp());
            case FRONT_COLLECT_CUBE: return Arrays.asList(operatorController.povRight());
            case FRONT_COLLECT_CONE: return Arrays.asList(operatorController.povLeft());
            case FLOOR_COLLECT: return Arrays.asList(operatorController.povDown());
            case HOME: return Arrays.asList(ALWAYS_FALSE); // TODO: figure out how to bind to home button
            case HOME_CLAW: return Arrays.asList(ALWAYS_FALSE); // TODO: figure out how to bind to share button
            default: return Arrays.asList(ALWAYS_FALSE);
        }
    }

    @Override
    public DoubleSupplier supplierFor(StickAction action) {
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
