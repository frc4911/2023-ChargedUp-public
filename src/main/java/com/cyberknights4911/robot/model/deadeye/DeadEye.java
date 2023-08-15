package com.cyberknights4911.robot.model.deadeye;

import java.util.function.Supplier;
import com.cyberknights4911.robot.auto.AutoCommandChooser;
import com.cyberknights4911.robot.drive.swerve.GyroIO;
import com.cyberknights4911.robot.drive.swerve.GyroIORealPigeon;
import com.cyberknights4911.robot.drive.swerve.SwerveIO;
import com.cyberknights4911.robot.drive.swerve.SwerveIOReal;
import com.cyberknights4911.robot.drive.swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj.RobotBase;
import libraries.cyberlib.drivers.CANCoderFactory;
import libraries.cyberlib.drivers.CtreError;
import libraries.cyberlib.drivers.Pigeon1Factory;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class DeadEye {
  private final AutoCommandChooser chooser;
  private final DeadEyeControllerBinding binding;
  private final SwerveSubsystem swerveSubsystem;

  public DeadEye() {
    TalonFXFactory roboRioTalonFactory = TalonFXFactory.createOnRoboRio();
    CANCoderFactory roboRioCANCoderFactory = CANCoderFactory.createOnRoboRio();
    Pigeon1Factory pigeon1Factory = Pigeon1Factory.createOnRoboRio();
    CtreError ctreError = new CtreError(DeadEyeConstants.LONG_CAN_TIMEOUTS_MS);

    chooser = new DeadEyeAutoCommandChooser();
    binding = new DeadEyeControllerBinding();
    Supplier<SwerveIO> frontLeftSwerveSupplier = SwerveIOReal.getSupplier(
      roboRioTalonFactory,
      roboRioCANCoderFactory,
      ctreError,
      DeadEyeConstants.Drive.FRONT_LEFT,
      DeadEyeConstants.Drive.SWERVE_DRIVE_CONSTANTS,
      DeadEyeConstants.Drive.PHYSICAL_SWERVE_MODULE);
    Supplier<SwerveIO> frontRightSwerveSupplier = SwerveIOReal.getSupplier(
      roboRioTalonFactory,
      roboRioCANCoderFactory,
      ctreError,
      DeadEyeConstants.Drive.FRONT_RIGHT,
      DeadEyeConstants.Drive.SWERVE_DRIVE_CONSTANTS,
      DeadEyeConstants.Drive.PHYSICAL_SWERVE_MODULE);
    Supplier<SwerveIO> backLeftSwerveSupplier = SwerveIOReal.getSupplier(
      roboRioTalonFactory,
      roboRioCANCoderFactory,
      ctreError,
      DeadEyeConstants.Drive.BACK_LEFT,
      DeadEyeConstants.Drive.SWERVE_DRIVE_CONSTANTS,
      DeadEyeConstants.Drive.PHYSICAL_SWERVE_MODULE);
    Supplier<SwerveIO> backRightSwerveSupplier = SwerveIOReal.getSupplier(
      roboRioTalonFactory,
      roboRioCANCoderFactory,
      ctreError,
      DeadEyeConstants.Drive.BACK_RIGHT,
      DeadEyeConstants.Drive.SWERVE_DRIVE_CONSTANTS,
      DeadEyeConstants.Drive.PHYSICAL_SWERVE_MODULE);
    Supplier<GyroIO> gyroSupplier = GyroIORealPigeon.getSupplier(
      pigeon1Factory, DeadEyeConstants.Drive.SWERVE_DRIVE_CONSTANTS, ctreError);

    swerveSubsystem = new SwerveSubsystem(
      GyroIO.create(RobotBase.isReal(), gyroSupplier),
      SwerveIO.create(RobotBase.isReal(), frontLeftSwerveSupplier),
      SwerveIO.create(RobotBase.isReal(), frontRightSwerveSupplier),
      SwerveIO.create(RobotBase.isReal(), backLeftSwerveSupplier),
      SwerveIO.create(RobotBase.isReal(), backRightSwerveSupplier),
      1, 2, 3, 4,
      DeadEyeConstants.Drive.SWERVE_DRIVE_CONSTANTS,
      DeadEyeConstants.Drive.PHYSICAL_SWERVE_MODULE);

      applyDefaultCommands();
    }

  private void applyDefaultCommands() {
    swerveSubsystem.setDefaultCommand(
      swerveSubsystem.createTeleopDriveCommand(binding)
    );
  }

}
