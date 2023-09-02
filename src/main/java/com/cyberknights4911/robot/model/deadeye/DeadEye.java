package com.cyberknights4911.robot.model.deadeye;

import org.littletonrobotics.junction.LoggedRobot;
import com.cyberknights4911.robot.RobotStateListener;
import com.cyberknights4911.robot.auto.AutoCommandHandler;
import com.cyberknights4911.robot.drive.swerve.GyroIORealPigeon;
import com.cyberknights4911.robot.drive.swerve.SwerveIOReal;
import com.cyberknights4911.robot.drive.swerve.SwerveModuleArgs;
import com.cyberknights4911.robot.drive.swerve.SwerveSubsystem;
import com.cyberknights4911.robot.drive.swerve.SwerveSubsystemArgs;
import edu.wpi.first.wpilibj.RobotBase;
import libraries.cyberlib.drivers.CANCoderFactory;
import libraries.cyberlib.drivers.CtreError;
import libraries.cyberlib.drivers.Pigeon1Factory;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class DeadEye implements RobotStateListener {
  private final AutoCommandHandler autoHandler;
  private final DeadEyeControllerBinding binding;
  private final SwerveSubsystem swerveSubsystem;

  public DeadEye() {
    TalonFXFactory roboRioTalonFactory = TalonFXFactory.createOnRoboRio();
    CANCoderFactory roboRioCANCoderFactory = CANCoderFactory.createOnRoboRio();
    Pigeon1Factory pigeon1Factory = Pigeon1Factory.createOnRoboRio();
    CtreError ctreError = new CtreError(DeadEyeConstants.LONG_CAN_TIMEOUTS_MS);

    autoHandler = new DeadEyeAutoCommandHandler();
    binding = new DeadEyeControllerBinding();

    swerveSubsystem = createSwerveSubsystem(roboRioTalonFactory, roboRioCANCoderFactory, pigeon1Factory, ctreError);

    applyDefaultCommands();
  }

  @Override
  public void onAutonomousInit(LoggedRobot robot) {
      autoHandler.startCurrentAutonomousCommand();
  }

  @Override
  public void onAutonomousExit(LoggedRobot robot) {
      autoHandler.stopCurrentAutonomousCommand();
  }

  private SwerveSubsystem createSwerveSubsystem(
    TalonFXFactory roboRioTalonFactory,
    CANCoderFactory roboRioCANCoderFactory,
    Pigeon1Factory pigeon1Factory,
    CtreError ctreError
  ) {
    SwerveModuleArgs.Builder frontLeftArgs = SwerveModuleArgs.builder()
      .setModuleNumber(0)
      .setSwerveDriveConstants(DeadEyeConstants.Drive.SWERVE_DRIVE_CONSTANTS)
      .setCotsConstants(DeadEyeConstants.Drive.PHYSICAL_SWERVE_MODULE);
    SwerveModuleArgs.Builder frontRightArgs = SwerveModuleArgs.builder()
      .setModuleNumber(1)
      .setSwerveDriveConstants(DeadEyeConstants.Drive.SWERVE_DRIVE_CONSTANTS)
      .setCotsConstants(DeadEyeConstants.Drive.PHYSICAL_SWERVE_MODULE);
    SwerveModuleArgs.Builder backLeftArgs = SwerveModuleArgs.builder()
      .setModuleNumber(2)
      .setSwerveDriveConstants(DeadEyeConstants.Drive.SWERVE_DRIVE_CONSTANTS)
      .setCotsConstants(DeadEyeConstants.Drive.PHYSICAL_SWERVE_MODULE);
    SwerveModuleArgs.Builder backRightArgs = SwerveModuleArgs.builder()
      .setModuleNumber(3)
      .setSwerveDriveConstants(DeadEyeConstants.Drive.SWERVE_DRIVE_CONSTANTS)
      .setCotsConstants(DeadEyeConstants.Drive.PHYSICAL_SWERVE_MODULE);
    SwerveSubsystemArgs.Builder swerveArgs = SwerveSubsystemArgs.builder()
      .setSwerveDriveConstants(DeadEyeConstants.Drive.SWERVE_DRIVE_CONSTANTS);
    if (RobotBase.isReal()) {
      frontLeftArgs.setSwerveIO(new SwerveIOReal(
        roboRioTalonFactory,
        roboRioCANCoderFactory,
        ctreError,
        DeadEyeConstants.Drive.FRONT_LEFT,
        DeadEyeConstants.Drive.SWERVE_DRIVE_CONSTANTS,
        DeadEyeConstants.Drive.PHYSICAL_SWERVE_MODULE));
      frontRightArgs.setSwerveIO(new SwerveIOReal(
        roboRioTalonFactory,
        roboRioCANCoderFactory,
        ctreError,
        DeadEyeConstants.Drive.FRONT_RIGHT,
        DeadEyeConstants.Drive.SWERVE_DRIVE_CONSTANTS,
        DeadEyeConstants.Drive.PHYSICAL_SWERVE_MODULE));
      backLeftArgs.setSwerveIO(new SwerveIOReal(
        roboRioTalonFactory,
        roboRioCANCoderFactory,
        ctreError,
        DeadEyeConstants.Drive.BACK_LEFT,
        DeadEyeConstants.Drive.SWERVE_DRIVE_CONSTANTS,
        DeadEyeConstants.Drive.PHYSICAL_SWERVE_MODULE));
      backRightArgs.setSwerveIO(new SwerveIOReal(
        roboRioTalonFactory,
        roboRioCANCoderFactory,
        ctreError,
        DeadEyeConstants.Drive.BACK_RIGHT,
        DeadEyeConstants.Drive.SWERVE_DRIVE_CONSTANTS,
        DeadEyeConstants.Drive.PHYSICAL_SWERVE_MODULE));
      swerveArgs.setGyroIO(new GyroIORealPigeon(pigeon1Factory, DeadEyeConstants.Drive.SWERVE_DRIVE_CONSTANTS, ctreError));
    }
    return new SwerveSubsystem(swerveArgs.build());
  }
  private void applyDefaultCommands() {
    swerveSubsystem.setDefaultCommand(
      swerveSubsystem.createTeleopDriveCommand(binding)
    );
  }

}
