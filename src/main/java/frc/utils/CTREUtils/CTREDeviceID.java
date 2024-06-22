package frc.utils.CTREUtils;

import frc.robot.constants.Phoenix6Constants;

public record CTREDeviceID(int ID, String bus) {

    public CTREDeviceID(int ID) {
        this(ID, Phoenix6Constants.CANBUS_NAME);
    }

}
