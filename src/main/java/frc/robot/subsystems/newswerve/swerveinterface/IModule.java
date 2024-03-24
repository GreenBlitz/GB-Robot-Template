package frc.robot.subsystems.newswerve.swerveinterface;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IModule {
    void setTargetOpenLoopVelocity(double voltage);

    void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond);


    void setTargetAngle(Rotation2d angle);

    void stop();

    void setBrake(boolean brake);

    void updateInputs(ModuleInputsAutoLogged inputs);
}
