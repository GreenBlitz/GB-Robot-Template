package frc.robot.constants;

import frc.robot.subsystems.swerve.modules.ModuleID;
import frc.utils.ctre.BusChain;
import frc.utils.ctre.CTREDeviceID;
public class IDs {

    public static final CTREDeviceID PIGEON_2_DEVICE_ID = new CTREDeviceID(0, BusChain.CANIVORE);

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

    public static class ModulesIDs {
        public static final ModuleID[] MODULE_IDS = {
                new ModuleID(
                        TalonFXIDs.FRONT_LEFT_STEER_MOTOR,
                        true,
                        TalonFXIDs.FRONT_LEFT_DRIVE_MOTOR,
                        false,
                        CANCodersIDs.FRONT_LEFT_ENCODER
                ),
                new ModuleID(
                        TalonFXIDs.FRONT_RIGHT_STEER_MOTOR,
                        true,
                        TalonFXIDs.FRONT_RIGHT_DRIVE_MOTOR,
                        true,
                        CANCodersIDs.FRONT_RIGHT_ENCODER
                ),
                new ModuleID(
                        TalonFXIDs.BACK_LEFT_STEER_MOTOR,
                        false,
                        TalonFXIDs.BACK_LEFT_DRIVE_MOTOR,
                        false,
                        CANCodersIDs.BACK_LEFT_ENCODER
                ),
                new ModuleID(
                        TalonFXIDs.BACK_RIGHT_STEER_MOTOR,
                        true,
                        TalonFXIDs.BACK_RIGHT_DRIVE_MOTOR,
                        false,
                        CANCodersIDs.BACK_RIGHT_ENCODER
                )
        };
    }

}
