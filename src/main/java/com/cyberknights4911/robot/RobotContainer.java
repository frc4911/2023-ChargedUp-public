// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.cyberknights4911.robot;

import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.subsystems.ArmSubsystem;
import com.cyberknights4911.robot.subsystems.BobSubsystem;
import com.cyberknights4911.robot.subsystems.ClawSubsystem;
import com.cyberknights4911.robot.subsystems.ClimberSubsystem;
import com.cyberknights4911.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import libraries.cheesylib.geometry.Pose2d;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final BobSubsystem bobSubsystem = new BobSubsystem();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // DRIVER
    //Bind reset IMU to Y
    driverController.y().onTrue(
      Commands.runOnce(() -> swerveSubsystem.setRobotPosition(Constants.ROBOT_STARTING_POSE))
      );
    // Bind open claw to right bumper
    driverController.rightBumper().onTrue(
      Commands.runOnce(() -> clawSubsystem.openClaw())
    );
    
    // Bind Right Trigger to collect cube
    driverController.rightTrigger().onTrue(
      Commands.sequence(
        Commands.runOnce(() -> clawSubsystem.openClaw(), clawSubsystem),
        Commands.runOnce(() -> clawSubsystem.closeClaw(), clawSubsystem)
        
        )
    );
    // Bind Left Trigger to collect cone
    driverController.leftTrigger().onTrue(
      Commands.sequence(
        Commands.runOnce(() -> clawSubsystem.openClaw(), clawSubsystem),
        Commands.runOnce(() -> clawSubsystem.closeClaw(), clawSubsystem)
        
        )
    );
    // Bind D-pad down to climb wheel lock
    driverController.povDown().onTrue(
      Commands.runOnce(() -> {
        // TODO: lock wheels
      })
    );
    // Bind Start to reset wheels
    driverController.povDown().onTrue(
      Commands.runOnce(() -> {
        // TODO: reset wheels
      })
    );

    // OPERATOR
    // Bind A to L2
    operatorController.a().onTrue(
      Commands.runOnce(()-> {
        // TODO move to L2
      }, armSubsystem)
    );
    // Bind X to L3
    operatorController.a().onTrue(
      Commands.runOnce(()-> {
        // TODO move to L3
      }, armSubsystem)
    );
    // Bind Y + Right Bumper to Climb Deploy
    operatorController.rightBumper().and(
      operatorController.y().onTrue(
        Commands.runOnce(() -> climberSubsystem.setExtended(true), climberSubsystem)
      )
    );
    // Bind D-pad up to stowed
    operatorController.povUp().onTrue(
      Commands.runOnce(() -> {
        // TODO()
      }, armSubsystem)
    );
    // Bind D-pad right to rear collect
    operatorController.povRight().onTrue(
      Commands.sequence(
        Commands.parallel(
          Commands.runOnce(() -> clawSubsystem.openClaw(), clawSubsystem),
          Commands.runOnce(() -> {
            // TODO move to rear collect
          }, armSubsystem)
        ),
        Commands.runOnce(() -> clawSubsystem.closeClaw(), clawSubsystem)
      )
    );
    // Bind D-pad left to front collect
    operatorController.povLeft().onTrue(
      Commands.sequence(
        Commands.parallel(
          Commands.runOnce(() -> clawSubsystem.openClaw(), clawSubsystem),
          Commands.runOnce(() -> {
            // TODO move to front collect
          }, armSubsystem)
        ),
        Commands.runOnce(() -> clawSubsystem.closeClaw(), clawSubsystem)
      )
    );
    // Bind D-pad down to floor collect
    operatorController.povDown().onTrue(
      Commands.sequence(
        Commands.parallel(
          Commands.runOnce(() -> clawSubsystem.openClaw(), clawSubsystem),
          Commands.runOnce(() -> {
            // TODO move to floor collect
          }, armSubsystem)
        ),
        Commands.runOnce(() -> clawSubsystem.closeClaw(), clawSubsystem)
      )
    );
    // Bind right trigger to retract Bob
    operatorController.rightTrigger().onTrue(
      Commands.runOnce(() -> bobSubsystem.setExtended(false), bobSubsystem)

    );
     // Bind left trigger to extend Bob
     operatorController.leftTrigger().onTrue(
      Commands.runOnce(() -> bobSubsystem.setExtended(true), bobSubsystem)
      
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;//Autos.exampleAuto(m_exampleSubsystem);
  }
}
