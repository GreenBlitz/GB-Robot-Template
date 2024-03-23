package frc.robot.constants;

public class Ports {

    public static class JoystickDriverStationPorts {

        public static final int MAIN = 0;

        public static final int SECOND = 1;

        public static final int THIRD = 2;

        public static final int FOURTH = 3;
    }

    public static class SwerveIDs {
        public static final int // CANCoder
                FRONT_LEFT_ENCODER_ID = 0,
                FRONT_RIGHT_ENCODER_ID = 1,
                BACK_LEFT_ENCODER_ID = 2,
                BACK_RIGHT_ENCODER_ID = 3;
        public static final int // TalonFX
                FRONT_LEFT_DRIVE_MOTOR_ID = 0,
                FRONT_RIGHT_DRIVE_MOTOR_ID = 1,
                BACK_LEFT_DRIVE_MOTOR_ID = 2,
                BACK_RIGHT_DRIVE_MOTOR_ID = 3;
        public static final int // TalonFX
                FRONT_LEFT_STEER_MOTOR_ID = 4,
                FRONT_RIGHT_STEER_MOTOR_ID = 5,
                BACK_LEFT_STEER_MOTOR_ID = 6,
                BACK_RIGHT_STEER_MOTOR_ID = 7;
    }

    public static final int PIGEON_ID = 0;

}
