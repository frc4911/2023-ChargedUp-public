package com.cyberknights4911.robot.constants;

public final class Ports {
    
    /** Private constructor to prevent instances. */
    private Ports() {}

    // Drive
    public static final int FRONT_RIGHT_DRIVE = 0;
    public static final int FRONT_RIGHT_STEER = 4;
    public static final int FRONT_LEFT_DRIVE  = 1;
    public static final int FRONT_LEFT_STEER  = 5;
    public static final int BACK_LEFT_DRIVE   = 2;
    public static final int BACK_LEFT_STEER   = 6;
    public static final int BACK_RIGHT_DRIVE  = 3;
    public static final int BACK_RIGHT_STEER  = 7;

    public static final int ROBOT_2022_FRONT_RIGHT_DRIVE = 1; // PDP 15
    public static final int ROBOT_2022_FRONT_RIGHT_STEER = 5; // PDP 10
    public static final int ROBOT_2022_FRONT_LEFT_DRIVE  = 2; // PDP 0
    public static final int ROBOT_2022_FRONT_LEFT_STEER  = 6; // PDP 4
    public static final int ROBOT_2022_BACK_LEFT_DRIVE   = 3; // PDP 1
    public static final int ROBOT_2022_BACK_LEFT_STEER   = 7; // PDP 5
    public static final int ROBOT_2022_BACK_RIGHT_DRIVE  = 4; // PDP 14
    public static final int ROBOT_2022_BACK_RIGHT_STEER  = 8; // PDP 11

    public static final int ROBOT_2022_FRONT_RIGHT_CANCODER = 0;
    public static final int ROBOT_2022_FRONT_LEFT_CANCODER  = 1;
    public static final int ROBOT_2022_BACK_LEFT_CANCODER   = 2;
    public static final int ROBOT_2022_BACK_RIGHT_CANCODER  = 3;

    // Arm Subsystem Motors
    public static final int SHOULDER_MOTOR_1 = 10;
    public static final int SHOULDER_MOTOR_2 = 11;
    public static final int SHOULDER_MOTOR_3 = 12;
    public static final int WRIST_MOTOR = 13;

    public static final int ARM_SHOULDER_ENCODER = 0;
    public static final int ARM_WRIST_ENCODER = 1;

    public static final int SLURPP_MOTOR = 20;

    public static final int BOB_SOLENOID_PORT = 0;
    public static final int BOB_MOTOR = 14;

    public static final int ROBOT_2022_HOOD_MOTOR = 12;

    // Climber Subsystem
    public static final int CLIMB_SOLENOID_PORT = 1;

    public static final int PIGEON = 0;
}
