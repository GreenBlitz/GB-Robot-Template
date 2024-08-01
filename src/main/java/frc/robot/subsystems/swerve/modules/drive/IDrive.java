package frc.robot.subsystems.swerve.modules.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.ModuleInputsContainer;

public interface IDrive {

    void setBrake(boolean brake);

    void stop();

    void runMotorByVoltage(double voltage);

    void setTargetClosedLoopVelocity(Rotation2d velocityPerSecond);

    void updateInputs(ModuleInputsContainer inputs);

}
