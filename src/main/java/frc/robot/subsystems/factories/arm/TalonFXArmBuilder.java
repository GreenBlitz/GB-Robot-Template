package frc.robot.subsystems.factories.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.subsystems.arm.Arm;

public class TalonFXArmBuilder {
    static Arm build(String logPath){


        return new Arm(logPath)
    }

}
