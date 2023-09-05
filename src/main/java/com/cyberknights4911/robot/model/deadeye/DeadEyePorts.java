package com.cyberknights4911.robot.model.deadeye;

public final class DeadEyePorts {

    private DeadEyePorts() {}
    
    public static final class Drive {
        private Drive() {}

        public static final int FRONT_LEFT_DRIVE = 1;
        public static final int FRONT_LEFT_STEER = 5;
        public static final int FRONT_RIGHT_DRIVE = 2; 
        public static final int FRONT_RIGHT_STEER = 6;
        public static final int BACK_LEFT_DRIVE = 3;
        public static final int BACK_LEFT_STEER = 7;
        public static final int BACK_RIGHT_DRIVE = 4;
        public static final int BACK_RIGHT_STEER = 8;

        public static final int FRONT_RIGHT_CANCODER = 0;
        public static final int FRONT_LEFT_CANCODER = 1;
        public static final int BACK_LEFT_CANCODER = 2;
        public static final int BACK_RIGHT_CANCODER = 3;

        public static final int PIGEON = 0;
    }

    public static final class Controller {
        private Controller() {}

        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
}
