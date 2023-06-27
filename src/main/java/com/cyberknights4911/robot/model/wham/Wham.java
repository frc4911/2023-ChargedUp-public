package com.cyberknights4911.robot.model.wham;

import com.cyberknights4911.robot.commands.MoveArmMotionMagicCommand;
import com.cyberknights4911.robot.commands.SlurppCommand;
import com.cyberknights4911.robot.commands.auto.AutoCommandChooser;
import com.cyberknights4911.robot.control.ControllerBinding;
import com.cyberknights4911.robot.model.RobotModel;
import com.cyberknights4911.robot.subsystems.Subsystems;
import com.cyberknights4911.robot.subsystems.arm.ArmPositions;

import edu.wpi.first.wpilibj2.command.Commands;

public final class Wham implements RobotModel {

  private final AutoCommandChooser autoCommandChooser;
  private final ControllerBinding<WhamButtonAction> binding;

    public Wham(Subsystems subsystems) {
      autoCommandChooser = new WhamAutoCommandChooser(subsystems);
      this.binding = new WhamControllerBinding();
    }

    @Override
    public void applyDefaultCommands(Subsystems subsystems) {

    // subsystems.getArmSubsystem().setDefaultCommand(
    //   MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED)
    // );

    // subsystems.getSlurppSubsystem().setDefaultCommand(
    //   Commands.runOnce(() -> {
    //     subsystems.getSlurppSubsystem().holdCurrentPosition();
    //   }, subsystems.getSlurppSubsystem())
    // );
    
        subsystems.getSwerveSubsystem().setDefaultCommand(
            subsystems.getSwerveSubsystem().createDefaultCommand(binding)
        );
        
    }

    @Override
    public void applyButtonActions(Subsystems subsystems) {
        binding.triggersFor(WhamButtonAction.RESET_IMU).onTrue(
            Commands.runOnce(
              () -> {
                subsystems.getSwerveSubsystem().zeroGyro();
                subsystems.getSwerveSubsystem().resetModulesToAbsolute();
              }
            )
          );
    
        binding.triggersFor(WhamButtonAction.SLURPP_BACKWARD_FAST).whileTrue(
          new SlurppCommand(subsystems.getSlurppSubsystem(), -0.85, subsystems.getArmSubsystem(), true));
    
        binding.triggersFor(WhamButtonAction.SLURPP_BACKWARD_SLOW).whileTrue(
          new SlurppCommand(subsystems.getSlurppSubsystem(), -0.4, subsystems.getArmSubsystem(), true));
    
        binding.triggersFor(WhamButtonAction.SLURPP_FORWARD_FAST).whileTrue(
            new SlurppCommand(subsystems.getSlurppSubsystem(), 0.85, subsystems.getArmSubsystem(), true));
    
        binding.triggersFor(WhamButtonAction.SLURPP_FORWARD_SLOW).whileTrue(
          new SlurppCommand(subsystems.getSlurppSubsystem(), 0.4, subsystems.getArmSubsystem(), true));
    
        binding.triggersFor(WhamButtonAction.COLLECT_SINGLE_SUBSTATION_FRONT).onTrue(
          MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_SINGLE_SUBSTATION_FRONT));
          
        binding.triggersFor(WhamButtonAction.COLLECT_SUBSTATION_FRONT).onTrue(
          MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_SUBSTATION_FRONT));
    
        binding.triggersFor(WhamButtonAction.COLLECT_SUBSTATION_BACK).onTrue(
          MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_SUBSTATION_BACK));
        
        binding.triggersFor(WhamButtonAction.COLLECT_FLOOR_BACK_CONE).onTrue(
          MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_BACK_CONE));
        
        binding.triggersFor(WhamButtonAction.COLLECT_FLOOR_BACK_CUBE).onTrue(
          MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_BACK_CUBE));
      
        binding.triggersFor(WhamButtonAction.COLLECT_FLOOR_FRONT_CONE).onTrue(
          MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_FRONT_CONE));
    
        binding.triggersFor(WhamButtonAction.COLLECT_FLOOR_FRONT_CUBE).onTrue(
          MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.COLLECT_FLOOR_FRONT_CUBE));
    
        binding.triggersFor(WhamButtonAction.STOW).onTrue(
          MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.STOWED));
    
        binding.triggersFor(WhamButtonAction.SCORE_L2).onTrue(
          MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.SCORE_L2));
    
        binding.triggersFor(WhamButtonAction.SCORE_L3).onTrue(
          MoveArmMotionMagicCommand.create(subsystems.getArmSubsystem(), ArmPositions.SCORE_L3));
    }

    @Override
    public AutoCommandChooser geAutoCommandChooser() {
      return autoCommandChooser;
    }
}
