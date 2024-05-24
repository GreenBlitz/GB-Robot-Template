package frc.robot.constants;

import frc.utils.CTREUtils.CTREDeviceID;

public class Ports {

    public static final int PIGEON_2_ID = 0;

    public static class JoystickDriverStationPorts {

        public static final int MAIN = 0;

        public static final int SECOND = 1;

        public static final int THIRD = 2;

        public static final int FOURTH = 3;

    }

    public static class TalonFXIDs {

        public static final CTREDeviceID FRONT_LEFT_STEER_MOTOR = new CTREDeviceID(0, Phoenix6Constants.CANIVORE_NAME);

        public static final CTREDeviceID FRONT_LEFT_DRIVE_MOTOR = new CTREDeviceID(1, Phoenix6Constants.CANIVORE_NAME);

        public static final CTREDeviceID FRONT_RIGHT_STEER_MOTOR = new CTREDeviceID(2, Phoenix6Constants.CANIVORE_NAME);

        public static final CTREDeviceID FRONT_RIGHT_DRIVE_MOTOR = new CTREDeviceID(3, Phoenix6Constants.CANIVORE_NAME);

        public static final CTREDeviceID BACK_LEFT_STEER_MOTOR = new CTREDeviceID(4, Phoenix6Constants.CANIVORE_NAME);

        public static final CTREDeviceID BACK_LEFT_DRIVE_MOTOR = new CTREDeviceID(5, Phoenix6Constants.CANIVORE_NAME);

        public static final CTREDeviceID BACK_RIGHT_STEER_MOTOR = new CTREDeviceID(6, Phoenix6Constants.CANIVORE_NAME);

        public static final CTREDeviceID BACK_RIGHT_DRIVE_MOTOR = new CTREDeviceID(7, Phoenix6Constants.CANIVORE_NAME);

    }

    public static class CANCodersIDs {

        public static final CTREDeviceID FRONT_LEFT_ENCODER = new CTREDeviceID(0, Phoenix6Constants.CANIVORE_NAME);

        public static final CTREDeviceID FRONT_RIGHT_ENCODER = new CTREDeviceID(1, Phoenix6Constants.CANIVORE_NAME);

        public static final CTREDeviceID BACK_LEFT_ENCODER = new CTREDeviceID(2, Phoenix6Constants.CANIVORE_NAME);

        public static final CTREDeviceID BACK_RIGHT_ENCODER = new CTREDeviceID(3, Phoenix6Constants.CANIVORE_NAME);

    }

}
