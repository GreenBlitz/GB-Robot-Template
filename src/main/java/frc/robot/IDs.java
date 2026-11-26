package frc.robot;

import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;

public class IDs {

  public static class TalonFXIDs {

    public static final Phoenix6DeviceID SWERVE_FRONT_LEFT_STEER =
        new Phoenix6DeviceID(0, BusChain.SYSTEMCORE_CAN_0);

    public static final Phoenix6DeviceID SWERVE_FRONT_LEFT_DRIVE =
        new Phoenix6DeviceID(1, BusChain.SYSTEMCORE_CAN_0);

    public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT_STEER =
        new Phoenix6DeviceID(2, BusChain.SYSTEMCORE_CAN_0);

    public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT_DRIVE =
        new Phoenix6DeviceID(3, BusChain.SYSTEMCORE_CAN_0);

    public static final Phoenix6DeviceID SWERVE_BACK_LEFT_STEER =
        new Phoenix6DeviceID(4, BusChain.SYSTEMCORE_CAN_0);

    public static final Phoenix6DeviceID SWERVE_BACK_LEFT_DRIVE =
        new Phoenix6DeviceID(5, BusChain.SYSTEMCORE_CAN_0);

    public static final Phoenix6DeviceID SWERVE_BACK_RIGHT_STEER =
        new Phoenix6DeviceID(6, BusChain.SYSTEMCORE_CAN_0);

    public static final Phoenix6DeviceID SWERVE_BACK_RIGHT_DRIVE =
        new Phoenix6DeviceID(7, BusChain.SYSTEMCORE_CAN_0);
  }

  public static class CANCoderIDs {

    public static final Phoenix6DeviceID SWERVE_FRONT_LEFT =
        new Phoenix6DeviceID(0, BusChain.SYSTEMCORE_CAN_0);

    public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT =
        new Phoenix6DeviceID(1, BusChain.SYSTEMCORE_CAN_0);

    public static final Phoenix6DeviceID SWERVE_BACK_LEFT =
        new Phoenix6DeviceID(2, BusChain.SYSTEMCORE_CAN_0);

    public static final Phoenix6DeviceID SWERVE_BACK_RIGHT =
        new Phoenix6DeviceID(3, BusChain.SYSTEMCORE_CAN_0);
  }

  public static class Pigeon2IDs {

    public static final Phoenix6DeviceID CHASSIS =
        new Phoenix6DeviceID(0, BusChain.SYSTEMCORE_CAN_0);
  }

  public static class CANdleIDs {}

  public static class SparkMAXIDs {}

  public static class DigitalInputsIDs {}
}
