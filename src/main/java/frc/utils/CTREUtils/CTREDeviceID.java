package frc.utils.CTREUtils;

import frc.robot.constants.Phoenix6Constants;

public class CTREDeviceID {

    private final int id;
    private final String bus;


    // Use the default bus name (empty string).
    public CTREDeviceID(int deviceNumber) {
        this(deviceNumber, Phoenix6Constants.CANBUS_NAME);
    }

    public CTREDeviceID(int deviceNumber, String bus) {
        this.id = deviceNumber;
        this.bus = bus;
    }

    public int getID() {
        return id;
    }

    public String getBus() {
        return bus;
    }

}
