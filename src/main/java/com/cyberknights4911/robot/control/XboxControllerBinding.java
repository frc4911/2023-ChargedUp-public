package com.cyberknights4911.robot.control;

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
    public Triggers triggersFor(ButtonAction action) {
        switch(action) {
            case ALIGN_COLLECT: return new Triggers();
            case RESET_IMU: return new Triggers(driverController.y());
            case SLURPP_BACKWARD_FAST: return new Triggers(driverController.rightTrigger());
            case SLURPP_FORWARD_FAST: return new Triggers( driverController.leftTrigger());
            case SLURPP_BACKWARD_SLOW: return new Triggers(driverController.rightBumper());
            case SLURPP_FORWARD_SLOW: return new Triggers(driverController.leftBumper());
            case CLIMB_WHEEL_LOCK: return new Triggers();
            case RESET_WHEELS: return new Triggers(driverController.start());
            case STOW: return new Triggers(operatorController.povDown(), driverController.povDown());
            case SCORE_L3: return new Triggers(operatorController.leftBumper(), driverController.x());
            case SCORE_L2: return new Triggers(operatorController.rightBumper(), driverController.a());
            case CLIMB_DEPLOY: return new Triggers();
            case CLIMB_LOCKOUT: return new Triggers();
            case BOB_STOW: return new Triggers();//Triggers(operatorController.rightTrigger());
            case BOB_DEPLOY: return new Triggers();//Triggers(operatorController.leftTrigger());
            case COLLECT_SUBSTATION_FRONT: return new Triggers(operatorController.povRight(), driverController.povLeft());
            case COLLECT_SUBSTATION_BACK: return new Triggers(operatorController.povLeft(), driverController.povRight());
            case COLLECT_SINGLE_SUBSTATION_FRONT: return new Triggers();
            case COLLECT_FLOOR_FRONT_CONE: return new Triggers(operatorController.a());
            case COLLECT_FLOOR_FRONT_CUBE: return new Triggers(operatorController.b());
            case COLLECT_FLOOR_BACK_CONE: return new Triggers();
            case COLLECT_FLOOR_BACK_CUBE: return new Triggers();
            case HOME: return new Triggers(ALWAYS_FALSE); // TODO: figure out how to bind to home button
            case HOME_CLAW: return new Triggers(ALWAYS_FALSE); // TODO: figure out how to bind to share button
            default: return new Triggers(ALWAYS_FALSE);
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
