package frc.robot.subsystems.swerve.modules.replay;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.IModule;
import frc.robot.subsystems.swerve.modules.inputs.ModuleInputsContainer;

public class EmptyModule implements IModule {

    @Override
    public void stop() {}

    @Override
    public void setBrake(boolean brake) {}

    @Override
    public void resetByEncoder() {}

    @Override
    public void runSteerMotorByVoltage(double voltage) {}

    @Override
    public void runDriveMotorByVoltage(double voltage) {}

    @Override
    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {}

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {}

    @Override
    public void setTargetAngle(Rotation2d angle) {}

    @Override
    public void updateInputs(ModuleInputsContainer inputs) {}

}
