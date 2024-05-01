package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IModule {

    void stop();

    void setBrake(boolean brake);

    void resetByEncoder();

    void setTargetOpenLoopVelocity(double voltage);

    void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond);

    void setTargetAngle(Rotation2d angle);

    void updateInputs(ModuleInputsAutoLogged inputs);

}
