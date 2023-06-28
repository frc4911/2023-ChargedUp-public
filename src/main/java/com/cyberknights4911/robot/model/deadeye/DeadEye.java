package com.cyberknights4911.robot.model.deadeye;

import com.cyberknights4911.robot.commands.auto.AutoCommandChooser;
import com.cyberknights4911.robot.control.ControllerBinding;
import com.cyberknights4911.robot.model.RobotModel;
import com.cyberknights4911.robot.subsystems.Subsystems;

import edu.wpi.first.wpilibj2.command.Commands;

public final class DeadEye implements RobotModel {

    private final AutoCommandChooser chooser;
    private final ControllerBinding<DeadEyeButtonAction> binding;

    public DeadEye() {
        this.binding = new DeadEyeControllerBinding();
        chooser = new DeadEyeAutoCommandChooser();
    }

    @Override
    public void applyDefaultCommands(Subsystems subsystems) {
        subsystems.getSwerveSubsystem().setDefaultCommand(
            subsystems.getSwerveSubsystem().createDefaultCommand(binding)
        );
    }

    @Override
    public void applyButtonActions(Subsystems subsystems) {
        binding.triggersFor(DeadEyeButtonAction.RESET_IMU).onTrue(
            Commands.runOnce(
              () -> {
                // subsystems.getSwerveSubsystem().zeroGyro();
                // subsystems.getSwerveSubsystem().resetModulesToAbsolute();
              }
            )
          );
    }

    @Override
    public AutoCommandChooser geAutoCommandChooser() {
        return chooser;
    }
}
