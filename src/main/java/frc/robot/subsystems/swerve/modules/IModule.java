package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.inputs.ModuleInputsContainer;

public interface IModule {

    void stop();

    void setBrake(boolean brake);

    void resetByEncoder();

    void runSteerMotorByVoltage(double voltage);

    void runDriveMotorByVoltage(double voltage);

    void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond);

    void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond);

    void setTargetAngle(Rotation2d angle);

    void updateInputs(ModuleInputsContainer inputs);

}
