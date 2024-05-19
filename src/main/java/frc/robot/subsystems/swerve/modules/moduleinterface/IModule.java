package frc.robot.subsystems.swerve.modules.moduleinterface;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IModule {

    void stop();

    void setBrake(boolean brake);

    void resetByEncoder();

    void runSteerMotorByVoltage(double voltage);

    void runDriveMotorByVoltage(double voltage);

    void setTargetOpenLoopVelocity(double targetVelocityMeterPerSecond);

    void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond);

    void setTargetAngle(Rotation2d angle);

    void updateInputs(ModuleInputsAutoLogged inputs);

}
