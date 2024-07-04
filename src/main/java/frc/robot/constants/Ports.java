package frc.robot.constants;


import frc.utils.ctre.BusChain;
import frc.utils.ctre.CTREDeviceID;

public class Ports {

    public static final int PIGEON_2_ID = 0;

    public static class TalonFXIDs {

        public static final CTREDeviceID FRONT_LEFT_STEER_MOTOR = new CTREDeviceID(0, BusChain.CANIVORE);

        public static final CTREDeviceID FRONT_LEFT_DRIVE_MOTOR = new CTREDeviceID(1, BusChain.CANIVORE);

        public static final CTREDeviceID FRONT_RIGHT_STEER_MOTOR = new CTREDeviceID(2, BusChain.CANIVORE);

        public static final CTREDeviceID FRONT_RIGHT_DRIVE_MOTOR = new CTREDeviceID(3, BusChain.CANIVORE);

        public static final CTREDeviceID BACK_LEFT_STEER_MOTOR = new CTREDeviceID(4, BusChain.CANIVORE);

        public static final CTREDeviceID BACK_LEFT_DRIVE_MOTOR = new CTREDeviceID(5, BusChain.CANIVORE);

        public static final CTREDeviceID BACK_RIGHT_STEER_MOTOR = new CTREDeviceID(6, BusChain.CANIVORE);

        public static final CTREDeviceID BACK_RIGHT_DRIVE_MOTOR = new CTREDeviceID(7, BusChain.CANIVORE);

    }

    public static class CANCodersIDs {

        public static final CTREDeviceID FRONT_LEFT_ENCODER = new CTREDeviceID(0, BusChain.CANIVORE);

        public static final CTREDeviceID FRONT_RIGHT_ENCODER = new CTREDeviceID(1, BusChain.CANIVORE);

        public static final CTREDeviceID BACK_LEFT_ENCODER = new CTREDeviceID(2, BusChain.CANIVORE);

        public static final CTREDeviceID BACK_RIGHT_ENCODER = new CTREDeviceID(3, BusChain.CANIVORE);

    }

}
