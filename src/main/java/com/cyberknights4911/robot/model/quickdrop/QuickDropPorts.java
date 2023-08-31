package com.cyberknights4911.robot.model.quickdrop;

public final class QuickDropPorts {

    private QuickDropPorts() {}

    public static final class Drive {
        private Drive() {}

        public static final int FRONT_RIGHT_DRIVE = 1; 
        public static final int FRONT_RIGHT_STEER = 5;
        public static final int FRONT_LEFT_DRIVE = 2;
        public static final int FRONT_LEFT_STEER = 6;
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

    public static final class Collector {
        private Collector() {}

        public static final int MOTOR = 20;
        public static final int SOLENOID = 2;
    }

    public static final class Indexer {
        private Indexer() {}

        public static final int MOTOR = 30;
        public static final int BEAM_BREAK_ENTER = 0;
        public static final int BEAM_BREAK_EXIT = 1;
    }


    public static final class Shooter {
        private Shooter() {}

        public static final int FLYWHEEL_LEFT_MOTOR = 10;
        public static final int FLYWHEEL_RIGHT_MOTOR = 11;
        public static final int HOOD_MOTOR = 12;
    }

    public static final class Controller {
        private Controller() {}

        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
}
