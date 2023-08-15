package com.cyberknights4911.robot.model.wham;

import java.util.function.DoubleSupplier;
import com.cyberknights4911.robot.control.ButtonBinding;
import com.cyberknights4911.robot.control.DriveStickAction;
import com.cyberknights4911.robot.control.StickBinding;
import com.cyberknights4911.robot.control.Triggers;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * WHAM default controller binding. Do not place alternate bindings in this class. Instead, create
 * a new ControllerBinding specific to the alternate control scheme and swap the binding out in
 * RobotContainer.
 */
public final class WhamControllerBinding implements ButtonBinding<WhamButtonAction>, StickBinding {
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    public WhamControllerBinding() {
        this(
            new CommandXboxController(WhamPorts.Controller.DRIVER_CONTROLLER_PORT),
            new CommandXboxController(WhamPorts.Controller.OPERATOR_CONTROLLER_PORT)
        );
    }

    public WhamControllerBinding(
        CommandXboxController driverController, CommandXboxController operatorController
    ) {
        this.driverController = driverController;
        this.operatorController = operatorController;
    }

    @Override
    public Triggers triggersFor(WhamButtonAction action) {
        switch(action) {
            case ALIGN_COLLECT: return new Triggers();
            case RESET_IMU: return new Triggers(driverController.y());
            case SLURPP_BACKWARD_FAST: return new Triggers(driverController.rightTrigger());
            case SLURPP_FORWARD_FAST: return new Triggers( driverController.leftTrigger());
            case SLURPP_BACKWARD_SLOW: return new Triggers(driverController.rightBumper());
            case SLURPP_FORWARD_SLOW: return new Triggers(driverController.leftBumper());
            case CLIMB_WHEEL_LOCK: return new Triggers(ALWAYS_FALSE); // Never implemented
            case RESET_WHEELS: return new Triggers(driverController.start());
            case STOW: return new Triggers(operatorController.povDown(), driverController.povDown());
            case SCORE_L3: return new Triggers(operatorController.leftBumper(), driverController.x());
            case SCORE_L2: return new Triggers(operatorController.rightBumper(), driverController.a());
            case CLIMB_DEPLOY: return new Triggers(ALWAYS_FALSE); // Never implemented
            case CLIMB_LOCKOUT: return new Triggers(ALWAYS_FALSE); // Never implemented
            case BOB_STOW: return new Triggers(ALWAYS_FALSE); // Never implemented
            case BOB_DEPLOY: return new Triggers(ALWAYS_FALSE); // Never implemented
            case COLLECT_SUBSTATION_FRONT: return new Triggers(operatorController.povRight(), driverController.povLeft());
            case COLLECT_SUBSTATION_BACK: return new Triggers(operatorController.povLeft(), driverController.povRight());
            case COLLECT_SINGLE_SUBSTATION_FRONT: return new Triggers();
            case COLLECT_FLOOR_FRONT_CONE: return new Triggers(operatorController.a(), driverController.povUp());
            case COLLECT_FLOOR_FRONT_CUBE: return new Triggers(operatorController.b());
            case COLLECT_FLOOR_BACK_CONE: return new Triggers(); // Never implemented
            case COLLECT_FLOOR_BACK_CUBE: return new Triggers(); // Never implemented
            case HOME: return new Triggers(ALWAYS_FALSE); // TODO: figure out how to bind to home button
            case HOME_CLAW: return new Triggers(ALWAYS_FALSE); // TODO: figure out how to bind to share button
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
