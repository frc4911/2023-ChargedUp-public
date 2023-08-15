package com.cyberknights4911.robot.model.wham;

import java.util.function.Supplier;
import org.littletonrobotics.junction.LoggedRobot;
import com.cyberknights4911.robot.RobotStateListener;
import com.cyberknights4911.robot.auto.AutoCommandChooser;
import com.cyberknights4911.robot.drive.swerve.GyroIO;
import com.cyberknights4911.robot.drive.swerve.GyroIORealPigeon;
import com.cyberknights4911.robot.drive.swerve.SwerveIO;
import com.cyberknights4911.robot.drive.swerve.SwerveIOReal;
import com.cyberknights4911.robot.drive.swerve.SwerveSubsystem;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import libraries.cyberlib.drivers.CANCoderFactory;
import libraries.cyberlib.drivers.CtreError;
import libraries.cyberlib.drivers.Pigeon2Factory;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class Wham implements RobotStateListener {
  private final SlurppSubsystem slurppSubsystem;
  private final ArmSubsystem armSubsystem ;
  private final SwerveSubsystem swerveSubsystem;

  private final AutoCommandChooser autoCommandChooser;
  private final WhamControllerBinding binding;

  private Command autonomousCommand;

  public Wham() {
    TalonFXFactory roboRioTalonFactory = TalonFXFactory.createOnRoboRio();
    TalonFXFactory canivoreTalonFactory = TalonFXFactory.createOnCanivore(WhamConstants.CANIVORE_NAME);
    CANCoderFactory roboRioCANCoderFactory = CANCoderFactory.createOnRoboRio();
    CANCoderFactory canivoreCANCoderFactory = CANCoderFactory.createOnCanivore(WhamConstants.CANIVORE_NAME);
    Pigeon2Factory pigeon2Factory = Pigeon2Factory.createOnRoboRio();
    CtreError ctreError = new CtreError(WhamConstants.LONG_CAN_TIMEOUTS_MS);

    Supplier<SlurppIO> slurppSupplier = SlurppIOReal.getSupplier(canivoreTalonFactory, ctreError);
    Supplier<ArmIO> armSupplier = ArmIOReal.getSupplier(canivoreTalonFactory, canivoreCANCoderFactory, ctreError);
    Supplier<SwerveIO> frontLeftSwerveSupplier = SwerveIOReal.getSupplier(
      roboRioTalonFactory,
      roboRioCANCoderFactory,
      ctreError,
      WhamConstants.Drive.FRONT_LEFT,
      WhamConstants.Drive.SWERVE_DRIVE_CONSTANTS,
      WhamConstants.Drive.PHYSICAL_SWERVE_MODULE);
    Supplier<SwerveIO> frontRightSwerveSupplier = SwerveIOReal.getSupplier(
      roboRioTalonFactory,
      roboRioCANCoderFactory,
      ctreError,
      WhamConstants.Drive.FRONT_RIGHT,
      WhamConstants.Drive.SWERVE_DRIVE_CONSTANTS,
      WhamConstants.Drive.PHYSICAL_SWERVE_MODULE);
    Supplier<SwerveIO> backLeftSwerveSupplier = SwerveIOReal.getSupplier(
      roboRioTalonFactory,
      roboRioCANCoderFactory,
      ctreError,
      WhamConstants.Drive.BACK_LEFT,
      WhamConstants.Drive.SWERVE_DRIVE_CONSTANTS,
      WhamConstants.Drive.PHYSICAL_SWERVE_MODULE);
    Supplier<SwerveIO> backRightSwerveSupplier = SwerveIOReal.getSupplier(
      roboRioTalonFactory,
      roboRioCANCoderFactory,
      ctreError,
      WhamConstants.Drive.BACK_RIGHT,
      WhamConstants.Drive.SWERVE_DRIVE_CONSTANTS,
      WhamConstants.Drive.PHYSICAL_SWERVE_MODULE);
    Supplier<GyroIO> gyroSupplier = GyroIORealPigeon.getSupplier(
      pigeon2Factory, WhamConstants.Drive.SWERVE_DRIVE_CONSTANTS, ctreError);

    slurppSubsystem = new SlurppSubsystem(SlurppIO.create(RobotBase.isReal(), slurppSupplier));
    armSubsystem = new ArmSubsystem(ArmIO.create(RobotBase.isReal(), armSupplier));
    swerveSubsystem = new SwerveSubsystem(
      GyroIO.create(RobotBase.isReal(), gyroSupplier),
      SwerveIO.create(RobotBase.isReal(), frontLeftSwerveSupplier),
      SwerveIO.create(RobotBase.isReal(), frontRightSwerveSupplier),
      SwerveIO.create(RobotBase.isReal(), backLeftSwerveSupplier),
      SwerveIO.create(RobotBase.isReal(), backRightSwerveSupplier),
      1, 2, 3, 4,
      WhamConstants.Drive.SWERVE_DRIVE_CONSTANTS,
      WhamConstants.Drive.PHYSICAL_SWERVE_MODULE);

    autoCommandChooser = new WhamAutoCommandChooser(armSubsystem, slurppSubsystem, swerveSubsystem);

    binding = new WhamControllerBinding();
    applyDefaultCommands();
    applyButtonActions();
  }

  @Override
  public void onAutonomousInit(LoggedRobot robot) {
    autonomousCommand = autoCommandChooser.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void onAutonomousExit(LoggedRobot robot) {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
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
