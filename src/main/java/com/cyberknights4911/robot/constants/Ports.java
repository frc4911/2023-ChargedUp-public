package com.cyberknights4911.robot.constants;

public final class Ports {
    private Ports() {}

    public static final class Drive {
        private Drive() {}
        public static final int FRONT_RIGHT_DRIVE = 2; // PDP 15
        public static final int FRONT_RIGHT_STEER = 6; // PDP 10
        public static final int FRONT_LEFT_DRIVE = 1; // PDP 0
        public static final int FRONT_LEFT_STEER = 5; // PDP 4
        public static final int BACK_LEFT_DRIVE = 3; // PDP 1
        public static final int BACK_LEFT_STEER = 7; // PDP 5
        public static final int BACK_RIGHT_DRIVE = 4; // PDP 14
        public static final int BACK_RIGHT_STEER = 8; // PDP 11
        public static final int FRONT_RIGHT_CANCODER = 0;
        public static final int FRONT_LEFT_CANCODER = 1;
        public static final int BACK_LEFT_CANCODER = 2;
        public static final int BACK_RIGHT_CANCODER = 3;

        public static final int PIGEON = 0;
    }

    public static final class Robot2022Hood {
        private Robot2022Hood() {}

        public static final int MOTOR = 12;
    }
}
