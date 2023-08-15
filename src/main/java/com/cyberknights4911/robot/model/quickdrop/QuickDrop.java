package com.cyberknights4911.robot.model.quickdrop;

import java.util.function.Supplier;
import com.cyberknights4911.robot.auto.AutoCommandChooser;
import com.cyberknights4911.robot.drive.swerve.GyroIO;
import com.cyberknights4911.robot.drive.swerve.GyroIORealPigeon;
import com.cyberknights4911.robot.drive.swerve.SwerveIO;
import com.cyberknights4911.robot.drive.swerve.SwerveIOReal;
import com.cyberknights4911.robot.drive.swerve.SwerveSubsystem;
import com.cyberknights4911.robot.model.quickdrop.camera.CollectorCamera;
import com.cyberknights4911.robot.model.quickdrop.collector.Collector;
import com.cyberknights4911.robot.model.quickdrop.collector.CollectorIO;
import com.cyberknights4911.robot.model.quickdrop.collector.CollectorIOReal;
import com.cyberknights4911.robot.model.quickdrop.collector.QuickDropCollectorCommand;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import libraries.cyberlib.drivers.CANCoderFactory;
import libraries.cyberlib.drivers.CtreError;
import libraries.cyberlib.drivers.Pigeon2Factory;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class QuickDrop {

    private final AutoCommandChooser chooser;
    private final QuickDropControllerBinding binding;
    private final Collector collector;
    private final CollectorCamera collectorCamera;
    private final SwerveSubsystem swerveSubsystem;

    public QuickDrop() {
        TalonFXFactory roboRioTalonFactory = TalonFXFactory.createOnRoboRio();
        TalonFXFactory canivoreTalonFactory = TalonFXFactory.createOnCanivore(QuickDropConstants.CANIVORE_NAME);
        CANCoderFactory roboRioCANCoderFactory = CANCoderFactory.createOnRoboRio();
        Pigeon2Factory pigeon2Factory = Pigeon2Factory.createOnCanivore(QuickDropConstants.CANIVORE_NAME);
        CtreError ctreError = new CtreError(QuickDropConstants.LONG_CAN_TIMEOUTS_MS);

        Supplier<SwerveIO> frontLeftSwerveSupplier = SwerveIOReal.getSupplier(
          roboRioTalonFactory,
          roboRioCANCoderFactory,
          ctreError,
          QuickDropConstants.Drive.FRONT_LEFT,
          QuickDropConstants.Drive.SWERVE_DRIVE_CONSTANTS,
          QuickDropConstants.Drive.PHYSICAL_SWERVE_MODULE);
        Supplier<SwerveIO> frontRightSwerveSupplier = SwerveIOReal.getSupplier(
          roboRioTalonFactory,
          roboRioCANCoderFactory,
          ctreError,
          QuickDropConstants.Drive.FRONT_RIGHT,
          QuickDropConstants.Drive.SWERVE_DRIVE_CONSTANTS,
          QuickDropConstants.Drive.PHYSICAL_SWERVE_MODULE);
        Supplier<SwerveIO> backLeftSwerveSupplier = SwerveIOReal.getSupplier(
          roboRioTalonFactory,
          roboRioCANCoderFactory,
          ctreError,
          QuickDropConstants.Drive.BACK_LEFT,
          QuickDropConstants.Drive.SWERVE_DRIVE_CONSTANTS,
          QuickDropConstants.Drive.PHYSICAL_SWERVE_MODULE);
        Supplier<SwerveIO> backRightSwerveSupplier = SwerveIOReal.getSupplier(
          roboRioTalonFactory,
          roboRioCANCoderFactory,
          ctreError,
          QuickDropConstants.Drive.BACK_RIGHT,
          QuickDropConstants.Drive.SWERVE_DRIVE_CONSTANTS,
          QuickDropConstants.Drive.PHYSICAL_SWERVE_MODULE);
        Supplier<GyroIO> gyroSupplier = GyroIORealPigeon.getSupplier(
          pigeon2Factory, QuickDropConstants.Drive.SWERVE_DRIVE_CONSTANTS, ctreError);

        chooser = new QuickDropAutoCommandChooser();
        binding = new QuickDropControllerBinding();

        collector = new Collector(CollectorIO.create(RobotBase.isReal(), CollectorIOReal.getSupplier(canivoreTalonFactory)));
        collectorCamera = new CollectorCamera();

        swerveSubsystem = new SwerveSubsystem(
            GyroIO.create(RobotBase.isReal(), gyroSupplier),
            SwerveIO.create(RobotBase.isReal(), frontLeftSwerveSupplier),
            SwerveIO.create(RobotBase.isReal(), frontRightSwerveSupplier),
            SwerveIO.create(RobotBase.isReal(), backLeftSwerveSupplier),
            SwerveIO.create(RobotBase.isReal(), backRightSwerveSupplier),
            1, 2, 3, 4,
            QuickDropConstants.Drive.SWERVE_DRIVE_CONSTANTS,
            QuickDropConstants.Drive.PHYSICAL_SWERVE_MODULE);

            applyDefaultCommands();
            applyButtonActions();
    }

    private void applyDefaultCommands() {
        swerveSubsystem.setDefaultCommand(
            swerveSubsystem.createTeleopDriveCommand(binding)
        );
    }

    private  void applyButtonActions() {
        // binding.triggersFor(QuickDropButtonAction.RESET_IMU).onTrue(
        //     Commands.runOnce(
        //       () -> {
        //         swerveSubsystem.zeroGyro();
        //         swerveSubsystem.resetModulesToAbsolute();
        //       }
        //     )
        //   );
        binding.triggersFor(QuickDropButtonAction.COLLECTOR_RUN_FORWARD).whileTrue(
            new QuickDropCollectorCommand(collector, 0.5)
        );
        binding.triggersFor(QuickDropButtonAction.COLLECTOR_RUN_REVERSE).whileTrue(
            new QuickDropCollectorCommand(collector, -0.5)
        );
        binding.triggersFor(QuickDropButtonAction.COLLECTOR_EXTEND).onTrue(
            Commands.runOnce(
                () -> {
                    collector.extend();
                },
                collector)
        );
        binding.triggersFor(QuickDropButtonAction.COLLECTOR_RETRACT).onTrue(
            Commands.runOnce(
                () -> {
                    collector.retract();
                },
                collector)
        );
    }

    // @Override
    // public AutoCommandChooser geAutoCommandChooser() {
    //     return chooser;
    // }
}
