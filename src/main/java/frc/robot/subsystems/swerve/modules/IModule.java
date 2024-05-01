package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IModule {

    void setTargetOpenLoopVelocity(double voltage);

    void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond);

    void setTargetAngle(Rotation2d angle);

    void resetByEncoder();

    void stop();

    void setBrake(boolean brake);

    void updateInputs(ModuleInputsAutoLogged inputs);

}
