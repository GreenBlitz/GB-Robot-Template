package frc.robot.subsystems.swerve.replayswerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.swerveinterface.IModule;
import frc.robot.subsystems.swerve.swerveinterface.ModuleInputsAutoLogged;

public class ReplayModule implements IModule {
    @Override
    public void setTargetOpenLoopVelocity(double voltage) {
    }

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
    }

    @Override
    public void resetByEncoder() {
    }

    @Override
    public void stop() {
    }

    @Override
    public void setBrake(boolean brake) {
    }

    @Override
    public void updateInputs(ModuleInputsAutoLogged inputs) {
    }
}
