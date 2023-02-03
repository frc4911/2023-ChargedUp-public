// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.cyberknights4911.robot;

import com.cyberknights4911.robot.commands.DefaultSwerveCommand;
import com.cyberknights4911.robot.commands.MoveHoodCommand;
import com.cyberknights4911.robot.constants.Constants;
import com.cyberknights4911.robot.subsystems.Subsystems;
import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem;
import com.cyberknights4911.robot.subsystems.arm.ArmSubsystem.ArmPositions;
import com.cyberknights4911.robot.subsystems.bob.BobSubsystem;
import com.cyberknights4911.robot.subsystems.climber.ClimberSubsystem;
import com.cyberknights4911.robot.subsystems.drive.SwerveSubsystem;
import com.cyberknights4911.robot.subsystems.hood.HoodSubsystem;
import com.cyberknights4911.robot.subsystems.hood.HoodSubsystem.HoodPositions;
import com.cyberknights4911.robot.subsystems.slurpp.SlurppSubsystem;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here...
  private final CommandXboxController mXbox = new CommandXboxController(0);
  Trigger xButton = mXbox.x(); // Creates a new Trigger object for the `X` button on exampleCommandController
  Trigger yButton = mXbox.y(); // Creates a new Trigger object for the `X` button on exampleCommandController
  Trigger bButton = mXbox.b(); // Creates a new Trigger object for the `X` button on exampleCommandController


  /** The container for the robot. Contains subsystems, OI devices, and commands. */

    //CommandScheduler.getInstance().registerSubsystem(mHood);  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);
  
  private final Subsystems subsystems;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    subsystems = new Subsystems();


    //Uncomment this for any swerve
    subsystems.getSwerveSubsystem().setDefaultCommand(new DefaultSwerveCommand(
      subsystems.getSwerveSubsystem(), this::getForwardInput, this::getStrafeInput, this::getRotationInput));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    subsystems.getSwerveSubsystem().convertCancoderToFX();
    xButton.onTrue(new MoveHoodCommand(subsystems.getHoodSubsystem(), HoodPositions.STOWED));
    yButton.onTrue(new MoveHoodCommand(subsystems.getHoodSubsystem(), HoodPositions.H1));
    bButton.onTrue(new MoveHoodCommand(subsystems.getHoodSubsystem(), HoodPositions.H2));

  }

  private static double deadband(double value, double tolerance) {
    if (Math.abs(value) < tolerance)
        return 0.0;

    return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
  }

  private static double square(double value) {
    return Math.copySign(value * value, value);
  }

  private double getForwardInput() {
      return -square(deadband(mXbox.getLeftY(), 0.1));
  }

  private double getStrafeInput() {
      return -square(deadband(mXbox.getLeftX(), 0.1));
  }

  private double getRotationInput() {
      return -square(deadband(mXbox.getRightX(), 0.1));
  }
  
  private void configureBindings() {
   
    // DRIVER
    //Bind reset IMU to Y
    driverController.y().onTrue(
      Commands.runOnce(() -> subsystems.getSwerveSubsystem().setRobotPosition(Constants.ROBOT_STARTING_POSE))
      );
    // Bind open claw to right bumper
    driverController.rightBumper().onTrue(
      Commands.runOnce(() -> subsystems.getSlurppSubsystem().slurpp()));

    // Bind Right Trigger to collect cube
    driverController.rightTrigger().onTrue(
      Commands.sequence(
        Commands.runOnce(() -> subsystems.getSlurppSubsystem().slurpp(), subsystems.getSlurppSubsystem()),
        Commands.runOnce(() -> subsystems.getSlurppSubsystem().spit(), subsystems.getSlurppSubsystem())
        
        )
    );
    // Bind Left Trigger to collect cone
    driverController.leftTrigger().onTrue(
      Commands.sequence(
        Commands.runOnce(() -> subsystems.getSlurppSubsystem().slurpp(), subsystems.getSlurppSubsystem()),
        Commands.runOnce(() -> subsystems.getSlurppSubsystem().spit(), subsystems.getSlurppSubsystem())
        
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
        subsystems.getArmSubsystem().setDesiredPosition(ArmPositions.CUBE_LEVEL_2);
        subsystems.getArmSubsystem().moveShoulder();
      }, subsystems.getArmSubsystem())
    );
    // Bind X to L3
    operatorController.a().onTrue(
      Commands.runOnce(()-> {
        subsystems.getArmSubsystem().setDesiredPosition(ArmPositions.CUBE_LEVEL_3);
        subsystems.getArmSubsystem().moveShoulder();
      }, subsystems.getArmSubsystem())
    );
    // Bind Y + Right Bumper to Climb Deploy
    operatorController.rightBumper().and(
      operatorController.y().onTrue(
        Commands.runOnce(() -> subsystems.getClimberSubsystem().setExtended(true), subsystems.getClimberSubsystem())
      )
    );
    // Bind D-pad up to stowed
    operatorController.povUp().onTrue(
      Commands.runOnce(() -> {
        // TODO()
      }, subsystems.getArmSubsystem())
    );
    // Bind D-pad right to rear collect
    operatorController.povRight().onTrue(
      Commands.sequence(
        Commands.parallel(
          Commands.runOnce(() -> subsystems.getSlurppSubsystem().slurpp(), subsystems.getSlurppSubsystem()),
          Commands.runOnce(() -> {
            // TODO move to rear collect
          }, subsystems.getArmSubsystem())
        ),
        Commands.runOnce(() -> subsystems.getSlurppSubsystem().spit(), subsystems.getSlurppSubsystem())
      )
    );
    // Bind D-pad left to front collect
    operatorController.povLeft().onTrue(
      Commands.sequence(
        Commands.parallel(
          Commands.runOnce(() -> subsystems.getSlurppSubsystem().slurpp(), subsystems.getSlurppSubsystem()),
          Commands.runOnce(() -> {
            // TODO move to front collect
          }, subsystems.getArmSubsystem())
        ),
        Commands.runOnce(() -> subsystems.getSlurppSubsystem().spit(), subsystems.getSlurppSubsystem())
      )
    );
    // Bind D-pad down to floor collect
    operatorController.povDown().onTrue(
      Commands.sequence(
        Commands.parallel(
          Commands.runOnce(() -> subsystems.getSlurppSubsystem().slurpp(), subsystems.getSlurppSubsystem()),
          Commands.runOnce(() -> {
            // TODO move to floor collect
          }, subsystems.getArmSubsystem())
        ),
        Commands.runOnce(() -> subsystems.getSlurppSubsystem().spit(), subsystems.getSlurppSubsystem())
      )
    );
    // Bind right trigger to retract Bob
    operatorController.rightTrigger().onTrue(
      Commands.runOnce(() -> subsystems.getBobSubsystem().toggleBob(false), subsystems.getBobSubsystem())

    );
     // Bind left trigger to extend Bob
     operatorController.leftTrigger().onTrue(
      Commands.runOnce(() -> subsystems.getBobSubsystem().toggleBob(true), subsystems.getBobSubsystem())
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // This will load the file "Test.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Test", new PathConstraints(4, 3));
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("hood", new MoveHoodCommand(subsystems.getHoodSubsystem(), HoodPositions.H2));
    eventMap.put("hoodStow", new MoveHoodCommand(subsystems.getHoodSubsystem(), HoodPositions.STOWED));

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        subsystems.getSwerveSubsystem()::getPose, // Pose2d supplier
        subsystems.getSwerveSubsystem()::setRobotPosition, // Pose2d consumer, used to reset odometry at the beginning of auto
        subsystems.getSwerveSubsystem().getmKinematics(), // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        subsystems.getSwerveSubsystem()::setSwerveModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        subsystems.getSwerveSubsystem() // The drive subsystem. Used to properly set the requirements of path following commands
    );

    Command fullAuto = new InstantCommand(() -> subsystems.getSwerveSubsystem().setState(SwerveSubsystem.ControlState.PATH_FOLLOWING))
        .andThen(autoBuilder.fullAuto(pathGroup));

    //TODO: We might need to call a stop on this or set something to stop the robot after it runs.
    return fullAuto;
  }
}
