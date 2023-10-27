package com.cyberknights4911.robot.model.wham;

import org.littletonrobotics.junction.LoggedRobot;
import com.cyberknights4911.robot.RobotStateListener;
import com.cyberknights4911.robot.auto.AutoCommandHandler;
import com.cyberknights4911.robot.drive.swerve.GyroIORealPigeon;
import com.cyberknights4911.robot.drive.swerve.SwerveIOReal;
import com.cyberknights4911.robot.drive.swerve.SwerveModule;
import com.cyberknights4911.robot.drive.swerve.SwerveModuleArgs;
import com.cyberknights4911.robot.drive.swerve.SwerveSubsystem;
import com.cyberknights4911.robot.drive.swerve.SwerveSubsystemArgs;
import com.cyberknights4911.robot.model.wham.arm.ArmIO;
import com.cyberknights4911.robot.model.wham.arm.ArmIOReal;
import com.cyberknights4911.robot.model.wham.arm.ArmPositions;
import com.cyberknights4911.robot.model.wham.arm.ArmSubsystem;
import com.cyberknights4911.robot.model.wham.arm.MoveArmMotionMagicCommand;
import com.cyberknights4911.robot.model.wham.slurpp.SlurppCommand;
import com.cyberknights4911.robot.model.wham.slurpp.SlurppIO;
import com.cyberknights4911.robot.model.wham.slurpp.SlurppIOReal;
import com.cyberknights4911.robot.model.wham.slurpp.SlurppSubsystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import libraries.cyberlib.drivers.CANCoderFactory;
import libraries.cyberlib.drivers.CtreError;
import libraries.cyberlib.drivers.Pigeon2Factory;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class Wham implements RobotStateListener {
  private final SlurppSubsystem slurppSubsystem;
  private final ArmSubsystem armSubsystem ;
  private final SwerveSubsystem swerveSubsystem;

  private final AutoCommandHandler autoHandler;
  private final WhamControllerBinding binding;

  public Wham() {
    TalonFXFactory roboRioTalonFactory = TalonFXFactory.createOnRoboRio();
    TalonFXFactory canivoreTalonFactory = TalonFXFactory.createOnCanivore(WhamConstants.CANIVORE_NAME);
    CANCoderFactory roboRioCANCoderFactory = CANCoderFactory.createOnRoboRio();
    CANCoderFactory canivoreCANCoderFactory = CANCoderFactory.createOnCanivore(WhamConstants.CANIVORE_NAME);
    Pigeon2Factory pigeon2Factory = Pigeon2Factory.createOnRoboRio();
    CtreError ctreError = new CtreError(WhamConstants.LONG_CAN_TIMEOUTS_MS);

    slurppSubsystem = createSlurppSubsystem(canivoreTalonFactory, ctreError);
    armSubsystem = createArmSubsystem(canivoreTalonFactory, canivoreCANCoderFactory, ctreError);
    swerveSubsystem = createSwerveSubsystem(roboRioTalonFactory, roboRioCANCoderFactory, pigeon2Factory, ctreError);
    // autoHandler = new BlockPartyAutos(armSubsystem, slurppSubsystem, swerveSubsystem);
    autoHandler = new WhamAutoCommandHandler(armSubsystem, slurppSubsystem, swerveSubsystem);
    binding = new WhamControllerBinding();

    applyDefaultCommands();
    applyButtonActions();
  }

  private ArmSubsystem createArmSubsystem(TalonFXFactory canivoreTalonFactory, CANCoderFactory canivoreCANCoderFactory,
      CtreError ctreError) {
    if (RobotBase.isReal()) {
      return new ArmSubsystem(new ArmIOReal(canivoreTalonFactory, canivoreCANCoderFactory, ctreError));
    } else {
      return new ArmSubsystem(new ArmIO() {});
    }
  }

  private SlurppSubsystem createSlurppSubsystem(TalonFXFactory canivoreTalonFactory, CtreError ctreError) {
    if (RobotBase.isReal()) {
      return new SlurppSubsystem(new SlurppIOReal(canivoreTalonFactory, ctreError));
    } else {
      return new SlurppSubsystem(new SlurppIO() {});
    }
  }

  private SwerveSubsystem createSwerveSubsystem(
    TalonFXFactory roboRioTalonFactory,
    CANCoderFactory roboRioCANCoderFactory,
    Pigeon2Factory pigeon2Factory,
    CtreError ctreError
  ) {
    SwerveModuleArgs.Builder frontLeftArgs = SwerveModuleArgs.builder()
      .setModuleNumber(0)
      .setSwerveDriveConstants(WhamConstants.Drive.SWERVE_DRIVE_CONSTANTS)
      .setCotsConstants(WhamConstants.Drive.PHYSICAL_SWERVE_MODULE);
    SwerveModuleArgs.Builder frontRightArgs = SwerveModuleArgs.builder()
      .setModuleNumber(1)
      .setSwerveDriveConstants(WhamConstants.Drive.SWERVE_DRIVE_CONSTANTS)
      .setCotsConstants(WhamConstants.Drive.PHYSICAL_SWERVE_MODULE);
    SwerveModuleArgs.Builder backLeftArgs = SwerveModuleArgs.builder()
      .setModuleNumber(2)
      .setSwerveDriveConstants(WhamConstants.Drive.SWERVE_DRIVE_CONSTANTS)
      .setCotsConstants(WhamConstants.Drive.PHYSICAL_SWERVE_MODULE);
    SwerveModuleArgs.Builder backRightArgs = SwerveModuleArgs.builder()
      .setModuleNumber(3)
      .setSwerveDriveConstants(WhamConstants.Drive.SWERVE_DRIVE_CONSTANTS)
      .setCotsConstants(WhamConstants.Drive.PHYSICAL_SWERVE_MODULE);
    SwerveSubsystemArgs.Builder swerveArgs = SwerveSubsystemArgs.builder()
      .setSwerveDriveConstants(WhamConstants.Drive.SWERVE_DRIVE_CONSTANTS);
    if (RobotBase.isReal()) {
      frontLeftArgs.setSwerveIO(new SwerveIOReal(
        roboRioTalonFactory,
        roboRioCANCoderFactory,
        ctreError,
        WhamConstants.Drive.FRONT_LEFT,
        WhamConstants.Drive.SWERVE_DRIVE_CONSTANTS,
        WhamConstants.Drive.PHYSICAL_SWERVE_MODULE));
      frontRightArgs.setSwerveIO(new SwerveIOReal(
        roboRioTalonFactory,
        roboRioCANCoderFactory,
        ctreError,
        WhamConstants.Drive.FRONT_RIGHT,
        WhamConstants.Drive.SWERVE_DRIVE_CONSTANTS,
        WhamConstants.Drive.PHYSICAL_SWERVE_MODULE));
      backLeftArgs.setSwerveIO(new SwerveIOReal(
        roboRioTalonFactory,
        roboRioCANCoderFactory,
        ctreError,
        WhamConstants.Drive.BACK_LEFT,
        WhamConstants.Drive.SWERVE_DRIVE_CONSTANTS,
        WhamConstants.Drive.PHYSICAL_SWERVE_MODULE));
      backRightArgs.setSwerveIO(new SwerveIOReal(
        roboRioTalonFactory,
        roboRioCANCoderFactory,
        ctreError,
        WhamConstants.Drive.BACK_RIGHT,
        WhamConstants.Drive.SWERVE_DRIVE_CONSTANTS,
        WhamConstants.Drive.PHYSICAL_SWERVE_MODULE));
      swerveArgs.setGyroIO(new GyroIORealPigeon(pigeon2Factory, WhamConstants.Drive.SWERVE_DRIVE_CONSTANTS, ctreError));
    }
    
    swerveArgs.setFrontLeftSwerveModule(new SwerveModule(frontLeftArgs.build()))
      .setFrontRightSwerveModule(new SwerveModule(frontRightArgs.build()))
      .setBackLeftSwerveModule(new SwerveModule(backLeftArgs.build()))
      .setBackRightSwerveModule(new SwerveModule(backRightArgs.build()));
    return new SwerveSubsystem(swerveArgs.build());
  }

  @Override
  public void onAutonomousInit(LoggedRobot robot) {
    autoHandler.startCurrentAutonomousCommand();
  }

  @Override
  public void onAutonomousExit(LoggedRobot robot) {
    autoHandler.stopCurrentAutonomousCommand();
  }

  private void applyDefaultCommands() {
  // armSubsystem.setDefaultCommand(
  //   MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.STOWED)
  // );

  // slurppSubsystem.setDefaultCommand(
  //   Commands.runOnce(() -> {
  //     slurppSubsystem.holdCurrentPosition();
  //   }, slurppSubsystem)
  // );
  
      swerveSubsystem.setDefaultCommand(
          swerveSubsystem.createTeleopDriveCommand(binding)
      );
  }

  private void applyButtonActions() {
      binding.triggersFor(WhamButtonAction.RESET_IMU).onTrue(
          Commands.runOnce(
            () -> {
              swerveSubsystem.zeroGyro();
              swerveSubsystem.resetModulesToAbsolute();
            }
          )
        );
  
      binding.triggersFor(WhamButtonAction.SLURPP_BACKWARD_FAST).whileTrue(
        new SlurppCommand(slurppSubsystem, -0.85, armSubsystem, true));
  
      binding.triggersFor(WhamButtonAction.SLURPP_BACKWARD_SLOW).whileTrue(
        new SlurppCommand(slurppSubsystem, -0.4, armSubsystem, true));
  
      binding.triggersFor(WhamButtonAction.SLURPP_FORWARD_FAST).whileTrue(
          new SlurppCommand(slurppSubsystem, 0.85, armSubsystem, true));
  
      binding.triggersFor(WhamButtonAction.SLURPP_FORWARD_SLOW).whileTrue(
        new SlurppCommand(slurppSubsystem, 0.4, armSubsystem, true));
  
      binding.triggersFor(WhamButtonAction.COLLECT_SINGLE_SUBSTATION_FRONT).onTrue(
        MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_SINGLE_SUBSTATION_FRONT));
        
      binding.triggersFor(WhamButtonAction.COLLECT_SUBSTATION_FRONT).onTrue(
        MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_SUBSTATION_FRONT));
  
      binding.triggersFor(WhamButtonAction.COLLECT_SUBSTATION_BACK).onTrue(
        MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_SUBSTATION_BACK));
      
      binding.triggersFor(WhamButtonAction.COLLECT_FLOOR_BACK_CONE).onTrue(
        MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_FLOOR_BACK_CONE));
      
      binding.triggersFor(WhamButtonAction.COLLECT_FLOOR_BACK_CUBE).onTrue(
        MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_FLOOR_BACK_CUBE));
    
      binding.triggersFor(WhamButtonAction.COLLECT_FLOOR_FRONT_CONE).onTrue(
        MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_FLOOR_FRONT_CONE));
  
      binding.triggersFor(WhamButtonAction.COLLECT_FLOOR_FRONT_CUBE).onTrue(
        MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.COLLECT_FLOOR_FRONT_CUBE));
  
      binding.triggersFor(WhamButtonAction.STOW).onTrue(
        MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.STOWED));
  
      binding.triggersFor(WhamButtonAction.SCORE_L2).onTrue(
        MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.SCORE_L2));
  
      binding.triggersFor(WhamButtonAction.SCORE_L3).onTrue(
        MoveArmMotionMagicCommand.create(armSubsystem, ArmPositions.SCORE_L3));
  }
}
