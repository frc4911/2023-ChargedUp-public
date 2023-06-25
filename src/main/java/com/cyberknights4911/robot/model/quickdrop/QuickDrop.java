package com.cyberknights4911.robot.model.quickdrop;

import com.cyberknights4911.robot.commands.QuickDropCollectorCommand;
import com.cyberknights4911.robot.commands.auto.AutoCommandChooser;
import com.cyberknights4911.robot.control.ControllerBinding;
import com.cyberknights4911.robot.model.RobotModel;
import com.cyberknights4911.robot.subsystems.Subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class QuickDrop implements RobotModel {

    private final AutoCommandChooser chooser;
    private final ControllerBinding<QuickDropButtonAction> binding;

    public QuickDrop() {
        this.binding = new QuickDropControllerBinding();
        chooser = new QuickDropAutoCommandChooser();
    }

    @Override
    public void applyDefaultCommands(Subsystems subsystems) {
        subsystems.getSwerveSubsystem().setDefaultCommand(
            subsystems.getSwerveSubsystem().createDefaultCommand(binding)
        );
    }

    @Override
    public void applyButtonActions(Subsystems subsystems) {
        binding.triggersFor(QuickDropButtonAction.RESET_IMU).onTrue(
            Commands.runOnce(
              () -> {
                subsystems.getSwerveSubsystem().zeroGyro();
                subsystems.getSwerveSubsystem().resetModulesToAbsolute();
              }
            )
          );
        binding.triggersFor(QuickDropButtonAction.COLLECTOR_RUN_FORWARD).whileTrue(
            new QuickDropCollectorCommand(subsystems, 0.10)
        );
        binding.triggersFor(QuickDropButtonAction.COLLECTOR_RUN_REVERSE).whileTrue(
            new QuickDropCollectorCommand(subsystems, -0.10)
        );
        binding.triggersFor(QuickDropButtonAction.COLLECTOR_EXTEND).onTrue(
            Commands.runOnce(
                () -> {
                    subsystems.getCollector().extend();
                },
                subsystems.getCollector())
        );
        binding.triggersFor(QuickDropButtonAction.COLLECTOR_RETRACT).onTrue(
            Commands.runOnce(
                () -> {
                    subsystems.getCollector().retract();
                },
                subsystems.getCollector())
        );
    }

    @Override
    public AutoCommandChooser geAutoCommandChooser() {
        return chooser;
    }
}
