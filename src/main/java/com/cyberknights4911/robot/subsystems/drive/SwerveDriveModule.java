package com.cyberknights4911.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.cyberknights4911.robot.config.SwerveModuleConfiguration;
import libraries.cyberlib.utils.Angles;
import libraries.cyberlib.utils.Util;

public class SwerveDriveModule {

    public enum ControlState {
        /**
         * No motor outputs. This state is used robot is powered up and stopped by
         * Driver Station.
         */
        NEUTRAL,
        /**
         * Runs motors in response to inputs.
         */
        OPEN_LOOP
    }

    public final SwerveModuleConfiguration config;
    public final String moduleName;

    private final SwerveIO swerveIO;
    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
    private final PeriodicIO periodicIO = new PeriodicIO();
    private final double maxSpeedInMetersPerSecond;

    private ControlState controlState = ControlState.NEUTRAL;

    public SwerveDriveModule(
        SwerveIO swerveIO, SwerveModuleConfiguration constants, double maxSpeedInMetersPerSecond
    ) {
        this.swerveIO = swerveIO;
        this.config = constants;
        this.maxSpeedInMetersPerSecond = maxSpeedInMetersPerSecond;
        moduleName = String.format("%s %d", config.name, config.moduleId);
        periodicIO.moduleID = config.moduleId;

        System.out.println("SwerveDriveModule " + moduleName + "," + config.steerMotorSlot0Kp);
    }

    protected void convertCancoderToFX2(){
        convertCancoderToFX2(true);
    }

    protected void convertCancoderToFX2(boolean useCancoders){
        int limit = 500;
        // mCANCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 50);
        double lastFrameTimestamp = -1;
        double frameTimestamp;
        double position;
        int index = 0;
        double[] cancoderPositions = {-10000,-1000,-100,-1,100}; // just need to be different numbers
        boolean allDone = false;
        double cancoderDegrees=0;

        if (useCancoders){
            do{
                frameTimestamp = swerveIO.getLastTimeStamp();
                if (frameTimestamp != lastFrameTimestamp){
                    position = swerveIO.getAbsolutePosition();
                    cancoderPositions[(++index)%cancoderPositions.length]=position;
                    allDone = true;
                    for (int kk=0; kk<cancoderPositions.length;kk++){
                        if ( Math.abs(cancoderPositions[kk]-cancoderPositions[(kk+1)%cancoderPositions.length]) >1 ){
                            allDone=false;
                            break;
                        }
                    }
                    // System.out.println(limit+" ("+mModuleName+")"+": CANCoder last frame timestamp = "+ frameTimestamp + 
                    //                     " current time = "+Timer.getFPGATimestamp() +" pos="+position);
                    lastFrameTimestamp = frameTimestamp;
                }
                Timer.delay(.1);

            } while (!allDone && (limit-- > 0));
            
            // System.out.println(mModuleName+": allDone "+Arrays.toString(cancoderPositions));
            cancoderDegrees = cancoderPositions[0];
        }
        else{
            System.out.println(moduleName +" assuming wheels aligned to 0 degrees, not using CANCoders");
            cancoderDegrees = 0;
        }
        double fxTicksBefore = swerveIO.getTurnSensorPosition();
        double fxTicksTarget = degreesToEncUnits(cancoderDegrees);
        double fxTicksNow = fxTicksBefore;
        int loops = 0;
        final double acceptableTickErr = 10;

        while ((Math.abs(fxTicksNow - fxTicksTarget) > acceptableTickErr) && (loops < 5)) {
            swerveIO.setTurnSensorPosition(fxTicksTarget);
            Timer.delay(.1);
            fxTicksNow = swerveIO.getTurnPosition();
            loops++;
        }

        System.out.println(config.name + " cancoder degrees: " + cancoderDegrees +
                ",  fx encoder ticks (before, target, adjusted): (" + fxTicksBefore + "," + fxTicksTarget + ","
                + fxTicksNow + ") loops:" + loops);
    }

    // protected void convertCancoderToFX(){
    //     // multiple (usually 2) sets were needed to set new encoder value
    //     double fxTicksBefore = mSteerMotor.getSelectedSensorPosition();
    //     double cancoderDegrees = mCANCoder.getAbsolutePosition();
        
    //     int limit = 5;
    //     do{
    //         if (mCANCoder.getLastError() != ErrorCode.OK) {
    //             System.out.println("error reading cancoder. Trying again!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! value read was "
    //                     + cancoderDegrees);
    //             Timer.delay(.1);
    //             cancoderDegrees = mCANCoder.getAbsolutePosition();
    //         }
    //         else{
    //             break;
    //         }
    //     }while (limit-- > 0);

    //     double fxTicksTarget = degreesToEncUnits(cancoderDegrees);
    //     double fxTicksNow = fxTicksBefore;
    //     int loops = 0;
    //     final double acceptableTickErr = 10;

    //     while (Math.abs(fxTicksNow - fxTicksTarget) > acceptableTickErr && loops < 5) {
    //         mSteerMotor.setSelectedSensorPosition(fxTicksTarget, 0, 0);
    //         Timer.delay(.1);
    //         fxTicksNow = mSteerMotor.getSelectedSensorPosition();
    //         loops++;
    //     }

    //     System.out.println(mConfig.kName + " cancoder degrees: " + cancoderDegrees +
    //             ",  fx encoder ticks (before, target, adjusted): (" + fxTicksBefore + "," + fxTicksTarget + ","
    //             + fxTicksNow + ") loops:" + loops);
    // }

    /**
     * Gets the current state of the module.
     * <p>
     * 
     * @return The current state of the module.
     */
    public synchronized SwerveModuleState getState() {
        // Note that Falcon is contiguous, so it can be larger than 2pi. Convert to
        // encoder readings
        // to SI units and let Rotation2d normalizes the angle between 0 and 2pi.
        Rotation2d currentAngle = Rotation2d.fromDegrees(Math.toDegrees(encoderUnitsToRadians(periodicIO.steerPosition)));

        return new SwerveModuleState(encVelocityToMetersPerSecond(periodicIO.driveVelocity), currentAngle);
    }

    public synchronized SwerveModulePosition getPosition(){
        Rotation2d currentAngle = Rotation2d.fromDegrees(Math.toDegrees(encoderUnitsToRadians(periodicIO.steerPosition)));

        return new SwerveModulePosition(encoderUnitsToDistance(periodicIO.drivePosition), currentAngle);
    }

    /**
     * Sets the state for the module.
     * <p>
     * 
     * @param desiredState Desired state for the module.
     */
    public synchronized void setState(SwerveModuleState desiredState) {
        // Note that Falcon is contiguous, so it can be larger than 2pi. Convert to
        // encoder readings
        // to SI units and let Rotation2d normalizes the angle between 0 and 2pi.
        Rotation2d currentAngle = Rotation2d.fromDegrees(Math.toDegrees(encoderUnitsToRadians(periodicIO.steerPosition)));

        // Minimizing the change in heading for the swerve module potentially requires
        // reversing the
        // direction the wheel spins. Odometry will still be accurate as both steer
        // angle and wheel
        // speeds will have their signs "flipped."
        var state = SwerveModuleState.optimize(desiredState, currentAngle);

        // Converts the velocity in SI units (meters per second) to a voltage (as a
        // percentage)
        // for the motor controllers.
        setDriveOpenLoop(Math.min(state.speedMetersPerSecond, maxSpeedInMetersPerSecond));

        setReferenceAngle(state.angle.getRadians());
    }

    /**
     * Sets the reference angle for the steer motor
     * <p>
     * 
     * @param referenceAngleRadians goal angle in radians
     */
    private void setReferenceAngle(double referenceAngleRadians) {
        // Note that Falcon is contiguous, so it can be larger than 2pi.
        double currentAngleRadians = encoderUnitsToRadians(periodicIO.steerPosition);

        // Map onto (0, 2pi)
        double currentAngleRadiansMod = Angles.normalizeAngle(currentAngleRadians);
        referenceAngleRadians = Angles.normalizeAngle(referenceAngleRadians);

        // Get the shortest angular distance between current and reference angles.
        double shortestDistance = Angles.shortest_angular_distance(currentAngleRadiansMod, referenceAngleRadians);

        // Adjust by adding the shortest distance to current angle (which can be in
        // multiples of 2pi)
        double adjustedReferenceAngleRadians = currentAngleRadians + shortestDistance;

        // mPeriodicIO.steerControlMode = ControlMode.MotionMagic;
        periodicIO.steerControlMode = ControlMode.Position;
        periodicIO.steerDemand = radiansToEncoderUnits(adjustedReferenceAngleRadians);
    }

    /**
     * Sets the motor controller settings and values for the steer motor.
     * <p>
     * 
     * @param angularVelocityInRadiansPerSecond Normalized value
     */
    private void setSteerOpenLoop(double angularVelocityInRadiansPerSecond) {
        periodicIO.steerControlMode = ControlMode.PercentOutput;
        periodicIO.steerDemand = angularVelocityInRadiansPerSecond;
    }

    /**
     * Sets the motor controller settings and values for the Drive motor.
     * <p>
     * 
     * @param velocity Normalized value
     */
    private void setDriveOpenLoop(double velocity) {
        if (controlState != ControlState.OPEN_LOOP) {
            controlState = ControlState.OPEN_LOOP;
        }

        periodicIO.driveControlMode = ControlMode.PercentOutput;
        periodicIO.driveDemand = velocity;
    }

    // Steer motor
    private double encoderUnitsToRadians(double encUnits) {
        return encUnits / config.steerMotorTicksPerRadian;
    }

    private double radiansToEncoderUnits(double radians) {
        return radians * config.steerMotorTicksPerRadian;
    }

    private int degreesToEncUnits(double degrees) {
        return (int) radiansToEncoderUnits(Math.toRadians(degrees));
    }

    private double encUnitsToDegrees(double encUnits) {
        return Math.toDegrees(encoderUnitsToRadians(encUnits));
    }

    // Drive motor
    private double encoderUnitsToDistance(double ticks) {
        return ticks * config.driveTicksPerUnitDistance;
    }

    private double encoderUnitsToVelocity(double ticks) {
        return ticks * config.driveTicksPerUnitVelocity;
    }

    private double distanceToEncoderUnits(double distanceInMeters) {
        return distanceInMeters / config.driveTicksPerUnitDistance;
    }

    // private double encUnitsToInches(double encUnits) {
    // return Units.metersToInches(encoderUnitsToDistance(encUnits));
    // }
    //
    // private double inchesToEncUnits(double inches) {
    // return distanceToEncoderUnits(Units.inchesToMeters(inches));
    // }

    private double metersPerSecondToEncVelocity(double metersPerSecond) {
        return metersPerSecond / config.driveTicksPerUnitVelocity;
    }

    private double encVelocityToMetersPerSecond(double encUnitsPer100ms) {
        return encUnitsPer100ms * config.driveTicksPerUnitVelocity;
    }

    /**
     * Sets the mode of operation during neutral throttle output.
     * <p>
     * 
     * @param neutralMode The desired mode of operation when the Talon FX
     *                    Controller output throttle is neutral (ie brake/coast)
     **/
    public synchronized void setNeutralMode(NeutralMode neutralMode) {
        switch (neutralMode) {
            case Brake: {
                swerveIO.setDriveBrakeMode(true);
                return;
            }
            case Coast:
            default:{
                swerveIO.setDriveBrakeMode(false);
                return;
            }
        }
    }

    public synchronized void stop() {
        if (controlState != ControlState.NEUTRAL) {
            controlState = ControlState.NEUTRAL;
        }

        swerveIO.setDrive(ControlMode.PercentOutput, 0.0);
        swerveIO.setTurn(ControlMode.PercentOutput, 0.0);
        periodicIO.driveDemand = 0;
        periodicIO.steerDemand = 0;
        periodicIO.driveControlMode = ControlMode.PercentOutput;
        periodicIO.steerControlMode = ControlMode.PercentOutput;
    }

    public synchronized void readPeriodicInputs() {
        periodicIO.steerPosition = (int) swerveIO.getTurnPosition();
        periodicIO.drivePosition = (int) swerveIO.getDrivePosition();
        periodicIO.driveVelocity = swerveIO.getDriveVelocity();
        periodicIO.steerVelocity = swerveIO.getTurnVelocity();
        // mPeriodicIO.steerError = mSteerMotor.getClosedLoopError(0);

        // swerveIO.updateInputs(inputs);
        // Logger.getInstance().processInputs(moduleName, inputs);
    }

    public synchronized void writePeriodicOutputs() {
        switch (controlState) {
            case OPEN_LOOP:
                // don't move if throttle is 0
                if (Util.epsilonEquals(periodicIO.driveDemand, 0.0, config.driveDeadband)) {
                    stop();
                } else {
                    // TODO: rbrewer check this drive demand value. It seems to be being interpreted as percent output
                    // rather than meters per second
                    swerveIO.setDrive(periodicIO.driveControlMode, periodicIO.driveDemand);
                    swerveIO.setTurn(periodicIO.steerControlMode, periodicIO.steerDemand);
                }
                break;
            case NEUTRAL:
            default:
                swerveIO.setDrive(periodicIO.driveControlMode, periodicIO.driveDemand);
                swerveIO.setTurn(periodicIO.steerControlMode, periodicIO.steerDemand);
                break;
        }
        // SmartDashboard.putNumber(moduleName + " Steering", periodicIO.steerDemand);
        // SmartDashboard.putNumber(moduleName + " Drive", periodicIO.driveDemand);

    }

    public static class PeriodicIO {
        // Inputs
        public int moduleID;
        public int steerPosition;
        public int drivePosition;
        public double steerVelocity;
        public double driveVelocity;
        public double steerCurrent;
        public double driveCurrent;
        public double steerError;

        // Outputs are in units for the motor controller.
        public ControlMode steerControlMode = ControlMode.PercentOutput;
        public ControlMode driveControlMode = ControlMode.PercentOutput;
        public double steerDemand;
        public double driveDemand;
    }
}
