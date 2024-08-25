package frc.robot.hardware;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;

public record CloseLoopControl(ControlRequest controlRequest, int pidSlot, Rotation2d targetSetPoint, ControlState controlState) {}
