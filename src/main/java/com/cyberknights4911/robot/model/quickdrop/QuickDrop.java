package com.cyberknights4911.robot.model.quickdrop;

import org.littletonrobotics.junction.LoggedRobot;

import com.cyberknights4911.robot.RobotStateListener;
import com.cyberknights4911.robot.auto.AutoCommandHandler;
import com.cyberknights4911.robot.drive.swerve.GyroIORealPigeon;
import com.cyberknights4911.robot.drive.swerve.SwerveIOReal;
import com.cyberknights4911.robot.drive.swerve.SwerveModule;
import com.cyberknights4911.robot.drive.swerve.SwerveModuleArgs;
import com.cyberknights4911.robot.drive.swerve.SwerveSubsystem;
import com.cyberknights4911.robot.drive.swerve.SwerveSubsystemArgs;
import com.cyberknights4911.robot.model.quickdrop.camera.CollectorCamera;
import com.cyberknights4911.robot.model.quickdrop.collector.Collector;
import com.cyberknights4911.robot.model.quickdrop.collector.CollectorIO;
import com.cyberknights4911.robot.model.quickdrop.collector.CollectorIOReal;
import com.cyberknights4911.robot.model.quickdrop.collector.QuickDropCollectorCommand;
import com.cyberknights4911.robot.model.quickdrop.indexer.Indexer;
import com.cyberknights4911.robot.model.quickdrop.indexer.IndexerCommand;
import com.cyberknights4911.robot.model.quickdrop.indexer.IndexerIO;
import com.cyberknights4911.robot.model.quickdrop.indexer.IndexerIOReal;
import com.cyberknights4911.robot.model.quickdrop.shooter.Shooter;
import com.cyberknights4911.robot.model.quickdrop.shooter.ShooterIO;
import com.cyberknights4911.robot.model.quickdrop.shooter.ShooterIOReal;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import libraries.cyberlib.drivers.CANCoderFactory;
import libraries.cyberlib.drivers.CtreError;
import libraries.cyberlib.drivers.Pigeon2Factory;
import libraries.cyberlib.drivers.TalonFXFactory;

public final class QuickDrop implements RobotStateListener {

    private final AutoCommandHandler autoHandler;
    private final QuickDropControllerBinding binding;
    private final Collector collector;
    private final Indexer indexer;
    private final Shooter shooter;
    private final CollectorCamera collectorCamera;
    private final SwerveSubsystem swerveSubsystem;

    public QuickDrop() {
        TalonFXFactory canivoreTalonFactory = TalonFXFactory.createOnCanivore(QuickDropConstants.CANIVORE_NAME);
        TalonFXFactory rioTalonFactory = TalonFXFactory.createOnRoboRio();
        CANCoderFactory canivoreCANCoderFactory = CANCoderFactory.createOnCanivore(QuickDropConstants.CANIVORE_NAME);
        Pigeon2Factory pigeon2Factory = Pigeon2Factory.createOnCanivore(QuickDropConstants.CANIVORE_NAME);
        CtreError ctreError = new CtreError(QuickDropConstants.LONG_CAN_TIMEOUTS_MS);

        autoHandler = new QuickDropAutoCommandHandler();
        binding = new QuickDropControllerBinding();

        collectorCamera = new CollectorCamera();
        collector = createCollector(canivoreTalonFactory);
        indexer = createIndexer(rioTalonFactory);
        shooter = createShooter(rioTalonFactory, ctreError);
        swerveSubsystem = createSwerveSubsystem(canivoreTalonFactory, canivoreCANCoderFactory, pigeon2Factory, ctreError);

        applyDefaultCommands();
        applyButtonActions();
    }

    @Override
    public void onAutonomousInit(LoggedRobot robot) {
        autoHandler.startCurrentAutonomousCommand();
    }
  
    @Override
    public void onAutonomousExit(LoggedRobot robot) {
        autoHandler.stopCurrentAutonomousCommand();
    }
    
     private Collector createCollector(TalonFXFactory canivoreTalonFactory) {
        if (RobotBase.isReal()) {
            return new Collector(new CollectorIOReal(canivoreTalonFactory));
        } else {
            return new Collector(new CollectorIO() {});
        }
    }
    
    private Indexer createIndexer(TalonFXFactory talonFactory) {
       if (RobotBase.isReal()) {
           return new Indexer(new IndexerIOReal(talonFactory));
       } else {
           return new Indexer(new IndexerIO() {});
       }
   }
    
   private Shooter createShooter(TalonFXFactory talonFactory, CtreError ctreError) {
      if (RobotBase.isReal()) {
          return new Shooter(new ShooterIOReal(talonFactory, ctreError));
      } else {
          return new Shooter(new ShooterIO() {});
      }
  }

    private SwerveSubsystem createSwerveSubsystem(
        TalonFXFactory canivoreTalonFactory,
        CANCoderFactory canivoreCANCoderFactory,
        Pigeon2Factory pigeon2Factory,
        CtreError ctreError
    ) {
        SwerveModuleArgs.Builder frontLeftArgs = SwerveModuleArgs.builder()
            .setModuleNumber(1)
            .setSwerveDriveConstants(QuickDropConstants.Drive.SWERVE_DRIVE_CONSTANTS)
            .setCotsConstants(QuickDropConstants.Drive.PHYSICAL_SWERVE_MODULE);
        SwerveModuleArgs.Builder frontRightArgs = SwerveModuleArgs.builder()
            .setModuleNumber(2)
            .setSwerveDriveConstants(QuickDropConstants.Drive.SWERVE_DRIVE_CONSTANTS)
            .setCotsConstants(QuickDropConstants.Drive.PHYSICAL_SWERVE_MODULE);
        SwerveModuleArgs.Builder backLeftArgs = SwerveModuleArgs.builder()
            .setModuleNumber(3)
            .setSwerveDriveConstants(QuickDropConstants.Drive.SWERVE_DRIVE_CONSTANTS)
            .setCotsConstants(QuickDropConstants.Drive.PHYSICAL_SWERVE_MODULE);
        SwerveModuleArgs.Builder backRightArgs = SwerveModuleArgs.builder()
            .setModuleNumber(4)
            .setSwerveDriveConstants(QuickDropConstants.Drive.SWERVE_DRIVE_CONSTANTS)
            .setCotsConstants(QuickDropConstants.Drive.PHYSICAL_SWERVE_MODULE);
        SwerveSubsystemArgs.Builder swerveArgs = SwerveSubsystemArgs.builder()
            .setSwerveDriveConstants(QuickDropConstants.Drive.SWERVE_DRIVE_CONSTANTS);
        if (RobotBase.isReal()) {
            frontLeftArgs.setSwerveIO(new SwerveIOReal(
                canivoreTalonFactory,
                canivoreCANCoderFactory,
                ctreError,
                QuickDropConstants.Drive.FRONT_LEFT,
                QuickDropConstants.Drive.SWERVE_DRIVE_CONSTANTS,
                QuickDropConstants.Drive.PHYSICAL_SWERVE_MODULE));
            frontRightArgs.setSwerveIO(new SwerveIOReal(
                canivoreTalonFactory,
                canivoreCANCoderFactory,
                ctreError,
                QuickDropConstants.Drive.FRONT_RIGHT,
                QuickDropConstants.Drive.SWERVE_DRIVE_CONSTANTS,
                QuickDropConstants.Drive.PHYSICAL_SWERVE_MODULE));
            backLeftArgs.setSwerveIO(new SwerveIOReal(
                canivoreTalonFactory,
                canivoreCANCoderFactory,
                ctreError,
                QuickDropConstants.Drive.BACK_LEFT,
                QuickDropConstants.Drive.SWERVE_DRIVE_CONSTANTS,
                QuickDropConstants.Drive.PHYSICAL_SWERVE_MODULE));
            backRightArgs.setSwerveIO(new SwerveIOReal(
                canivoreTalonFactory,
                canivoreCANCoderFactory,
                ctreError,
                QuickDropConstants.Drive.BACK_RIGHT,
                QuickDropConstants.Drive.SWERVE_DRIVE_CONSTANTS,
                QuickDropConstants.Drive.PHYSICAL_SWERVE_MODULE));
            swerveArgs.setGyroIO(new GyroIORealPigeon(pigeon2Factory, QuickDropConstants.Drive.SWERVE_DRIVE_CONSTANTS, ctreError));
        }
        swerveArgs.setFrontLeftSwerveModule(new SwerveModule(frontLeftArgs.build()))
            .setFrontRightSwerveModule(new SwerveModule(frontRightArgs.build()))
            .setBackLeftSwerveModule(new SwerveModule(backLeftArgs.build()))
            .setBackRightSwerveModule(new SwerveModule(backRightArgs.build()));
        return new SwerveSubsystem(swerveArgs.build());
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

        binding.triggersFor(QuickDropButtonAction.INDEXER_RUN).onTrue(
            new IndexerCommand(indexer, 0.15)
        );

        Command spinFlywheel = Commands.runOnce(
            () -> {
                shooter.setShooterSpeed(12000);
            }, shooter);
        Command waitOne = Commands.waitSeconds(1.5);
        Command runIndexer = Commands.runOnce(
            () -> {
                indexer.run(0.3);
            }, indexer);
        Command waitTwo = Commands.waitSeconds(1.0);
        Command stopitnow = Commands.runOnce(
            () -> {
                indexer.run(0);
                shooter.setShooterSpeed(0);
            }, shooter, indexer);
        binding.triggersFor(QuickDropButtonAction.SHOOTER_BLAST).onTrue(
            spinFlywheel.andThen(waitOne).andThen(runIndexer).andThen(waitTwo).andThen(stopitnow)
        );
    }
}
