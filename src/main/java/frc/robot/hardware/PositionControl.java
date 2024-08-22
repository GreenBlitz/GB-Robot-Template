package frc.robot.hardware;

import edu.wpi.first.math.geometry.Rotation2d;

public class PositionControl {

    boolean isMM;
    int slot;
    Rotation2d targetPosition;


    public int getSlot() {
        return slot;
    }

    public boolean isMM() {
        return isMM;
    }

    public Rotation2d getTargetPosition() {
        return targetPosition;
    }

}
