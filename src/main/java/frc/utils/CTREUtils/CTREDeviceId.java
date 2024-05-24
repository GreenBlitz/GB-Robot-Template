package frc.utils.CTREUtils;

import frc.robot.constants.Phoenix6Constants;

public class CTREDeviceId {

    private final int id;
    private final String bus;


    // Use the default bus name (empty string).
    public CTREDeviceId(int deviceNumber) {
        this(deviceNumber, Phoenix6Constants.CANBUS_NAME);
    }

    public CTREDeviceId(int deviceNumber, String bus) {
        this.id = deviceNumber;
        this.bus = bus;
    }

    public int getDeviceNumber() {
        return id;
    }

    public String getBus() {
        return bus;
    }

}
