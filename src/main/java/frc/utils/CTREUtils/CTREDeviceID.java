package frc.utils.CTREUtils;

import frc.robot.constants.Phoenix6Constants;

public record CTREDeviceID(int id, String bus) {

    public CTREDeviceID(int id) {
        this(id, Phoenix6Constants.CANBUS_NAME);
    }

}
