package com.cyberknights4911.robot.constants;

import com.google.auto.value.AutoValue;

import edu.wpi.first.math.util.Units;

/* Contains values and required settings for common COTS swerve modules. */
@AutoValue
public abstract class CotsFalconSwerveConstants {
  public abstract double wheelDiameter();
  public abstract double angleGearRatio();
  public abstract double driveGearRatio();
  public abstract double angleKP();
  public abstract double angleKI();
  public abstract double angleKD();
  public abstract double angleKF();
  public abstract boolean driveMotorInvert();
  public abstract boolean angleMotorInvert();
  public abstract boolean canCoderInvert();

  public double wheelCircumference() {
    return wheelDiameter() * Math.PI;
  }

  static Builder builder() {
    return new AutoValue_CotsFalconSwerveConstants.Builder();
  }

  @AutoValue.Builder
  public abstract static class Builder {
    abstract Builder setWheelDiameter(double value);
    abstract Builder setAngleGearRatio(double value);
    abstract Builder setDriveGearRatio(double value);
    abstract Builder setAngleKP(double value);
    abstract Builder setAngleKI(double value);
    abstract Builder setAngleKD(double value);
    abstract Builder setAngleKF(double value);
    abstract Builder setDriveMotorInvert(boolean value);
    abstract Builder setAngleMotorInvert(boolean value);
    abstract Builder setCanCoderInvert(boolean value);
    abstract CotsFalconSwerveConstants build();
  }
    
  /** Swerve Drive Specialties - MK2 Module*/
  // TODO: verify these values
  public static CotsFalconSwerveConstants SDSMK2(double driveGearRatio) {
    return builder()
        .setWheelDiameter(Units.inchesToMeters(4.0))
        .setAngleGearRatio(18.0)
        .setDriveGearRatio(driveGearRatio)
        .setAngleKP(0.2)
        .setAngleKI(0.0)
        .setAngleKD(0.0)
        .setAngleKF(0.0)
        .setAngleMotorInvert(false)
        .setDriveMotorInvert(false)
        .setCanCoderInvert(false)
        .build();
    }
    
    /** Swerve Drive Specialties - MK3 Module*/
    public static CotsFalconSwerveConstants SDSMK3(double driveGearRatio) {
        return builder()
            .setWheelDiameter(Units.inchesToMeters(4.0))
            .setAngleGearRatio(12.8)
            .setDriveGearRatio(driveGearRatio)
            .setAngleKP(0.2)
            .setAngleKI(0.0)
            .setAngleKD(0.0)
            .setAngleKF(0.0)
            .setAngleMotorInvert(false)
            .setDriveMotorInvert(false)
            .setCanCoderInvert(false)
            .build();
    }

    /** Swerve Drive Specialties - MK4 Module*/
    public static CotsFalconSwerveConstants SDSMK4(double driveGearRatio) {
        return builder()
            .setWheelDiameter(Units.inchesToMeters(4.0))
            .setAngleGearRatio(12.8)
            .setDriveGearRatio(driveGearRatio)
            .setAngleKP(0.2)
            .setAngleKI(0.0)
            .setAngleKD(0.0)
            .setAngleKF(0.0)
            .setAngleMotorInvert(false)
            .setDriveMotorInvert(false)
            .setCanCoderInvert(false)
            .build();
    }

    /** Swerve Drive Specialties - MK4i Module*/
    public static CotsFalconSwerveConstants SDSMK4i(double driveGearRatio) {
        return builder()
            .setWheelDiameter(Units.inchesToMeters(4.0))
            .setAngleGearRatio((150.0 / 7.0))// (150 / 7) : 1
            .setDriveGearRatio(driveGearRatio)
            .setAngleKP(0.3)
            .setAngleKI(0.0)
            .setAngleKD(0.0)
            .setAngleKF(0.0)
            .setAngleMotorInvert(true)
            .setDriveMotorInvert(false)
            .setCanCoderInvert(false)
            .build();
    }

    /* Drive Gear Ratios for all supported modules */
    public final class DriveGearRatios {

        private DriveGearRatios() {}

        
        /** SDS MK2 - ????????? */
        // TODO: fix this
        public static final double SDSMK2 = (8.16);

        /* SDS MK3 */
        /** SDS MK3 - 8.16 : 1 */
        public static final double SDSMK3_Standard = (8.16);
        /** SDS MK3 - 6.86 : 1 */
        public static final double SDSMK3_Fast = (6.86);

        /* SDS MK4 */
        /** SDS MK4 - 8.14 : 1 */
        public static final double SDSMK4_L1 = (8.14);
        /** SDS MK4 - 6.75 : 1 */
        public static final double SDSMK4_L2 = (6.75);
        /** SDS MK4 - 6.12 : 1 */
        public static final double SDSMK4_L3 = (6.12);
        /** SDS MK4 - 5.14 : 1 */
        public static final double SDSMK4_L4 = (5.14);
        
        /* SDS MK4i */
        /** SDS MK4i - 8.14 : 1 */
        public static final double SDSMK4i_L1 = (8.14);
        /** SDS MK4i - 6.75 : 1 */
        public static final double SDSMK4i_L2 = (6.75);
        /** SDS MK4i - 6.12 : 1 */
        public static final double SDSMK4i_L3 = (6.12);
    }
}
