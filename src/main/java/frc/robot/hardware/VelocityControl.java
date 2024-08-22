package frc.robot.hardware;

import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Function;

public class VelocityControl {

    enum Troll {
        Velocity(velocityVoltage -> velocityVoltage.)

        Troll(Function<>)
    }

    boolean isMM;
    int slot;
    Rotation2d targetVelocity;


    public int getSlot() {
        return slot;
    }

    public boolean isMM() {
        return isMM;
    }

    public Rotation2d getTargetVelocity() {
        return targetVelocity;
    }

}
