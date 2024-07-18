package frc.robot.subsystems.swerve.modules;

import frc.utils.ctre.CTREDeviceID;

public record ModuleID (
        CTREDeviceID steerMotorDeviceID, boolean isSteerMotorInverted,
        CTREDeviceID driveMotorDeviceID, boolean isDriveMotorInverted,
        CTREDeviceID encoderDeviceID
) {}
