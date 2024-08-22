package frc.robot.hardware.cansparkmax;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.BiFunction;
import java.util.function.Function;

public class ConstantsNeo {

    private BiFunction<Rotation2d, Rotation2d, Rotation2d> ff;
    private int defPosSlot = -1;
    private int defVelocitySlot = -1;

    public int getDefPosSlot() {
        return defPosSlot;
    }

    public BiFunction<Rotation2d, Rotation2d, Rotation2d> getFf() {
        return ff;
    }

    public int getDefVelocitySlot() {
        return defVelocitySlot;
    }

}
