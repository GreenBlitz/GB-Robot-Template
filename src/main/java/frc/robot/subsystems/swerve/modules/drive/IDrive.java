package frc.robot.subsystems.swerve.modules.drive;

import frc.robot.subsystems.swerve.modules.inputs.ModuleInputsContainer;

public interface IDrive {

    void stop();

    void setBrake(boolean brake);

    void runMotorByVoltage(double voltage);

    void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond);

    void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond);

    void updateInputs(ModuleInputsContainer inputs);

}
