package frc.robot.subsystems.swerve.module.factory;

public record RealModuleConstants(String logPath, double maxDriveVelocityMPS, double wheelDiameterMeters, int driveMotorId, int steerMotorId, int encoderId, boolean isCTRE) {
}
