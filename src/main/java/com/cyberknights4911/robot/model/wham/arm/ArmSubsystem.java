package com.cyberknights4911.robot.model.wham.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling the arm.
 */
public final class ArmSubsystem extends SubsystemBase {
    public static final double SHOULDER_GEAR_RATIO = 120.0;
    public static final double WRIST_GEAR_RATIO = 60.0;
    public static final double TICKS_PER_REVOLUTION = 4096;
    public static final double DEGREES_PER_REVOLUTION = 360;

    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private final Mechanism2d mechanism2d = new Mechanism2d(3, 3);
    private final MechanismRoot2d root = mechanism2d.getRoot("root", 1.5, 0);
    private final MechanismLigament2d yeet = new MechanismLigament2d( "yeet", 1, 0);
    private final MechanismLigament2d slurpp = new MechanismLigament2d("slurpp", 0.25, 0);

    public ArmSubsystem(ArmIO armIO) {
        super();
        this.armIO = armIO;
        reset();
        root.append(yeet);
        yeet.append(slurpp);
    }

    public void reset() {
    }

    public void setBrakeMode() {
        setShoulderBrakeMode();
        setWristBrakeMode();
    }

    public void setShoulderBrakeMode() {
        armIO.setShoulderOutput(0);
        armIO.setShoulderBrakeMode();
    }

    public void setWristBrakeMode() {
        armIO.setWristOutput(0);
        armIO.setWristBrakeMode();
    }

    public boolean moveShoulder(double desiredShoulderPosition) {
        armIO.setShoulderPosition(convertDegreesToCtreTicks(desiredShoulderPosition));
        return true;
    }

    public boolean moveWrist(double desiredWristPosition) {
        armIO.setWristPosition(convertDegreesToCtreTicks(desiredWristPosition));
        return true;
    }

    public void resetCANCoders() {
        armIO.offsetShoulder();
        armIO.offsetWrist();
    }
    @Override
    public void periodic() {
        armIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm", inputs);
        yeet.setAngle(getShoulderPositionDegrees());
        slurpp.setAngle(getWristPositionDegrees());
        Logger.getInstance().recordOutput("Arm", mechanism2d);
    }

    private double getWristPositionDegrees() {
        return armIO.getWristEncoderDegrees();
    }

    public double getShoulderPositionDegrees() {
        return armIO.getShoulderEncoderDegrees();
    }

    public double getShoulderTrajectoryPosition() {
        return armIO.getShoulderTrajectoryPosition();
    }

    public double getWristTrajectoryPosition() {
        return armIO.getWristTrajectoryPosition();
    }

    public static double convertDegreesToCtreTicks(double degrees) {
        return degrees * TICKS_PER_REVOLUTION / DEGREES_PER_REVOLUTION;
    }

    public static double convertCtreTicksToDegrees(double ticks) {
        return ticks / TICKS_PER_REVOLUTION * DEGREES_PER_REVOLUTION;
    }
}
