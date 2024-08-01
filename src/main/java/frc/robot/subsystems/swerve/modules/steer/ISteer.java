package frc.robot.subsystems.swerve.modules.steer;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.inputs.ModuleInputsContainer;

public interface ISteer {

    void stop();

    void setBrake(boolean brake);

    void resetToAngle(Rotation2d angle);

    void runMotorByVoltage(double voltage);

    void setTargetAngle(Rotation2d angle);

    void updateInputs(ModuleInputsContainer inputs);

}
