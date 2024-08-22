package frc.robot.hardware;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Supplier;


public record CloseLoopControl(ControlRequest controlRequest, int pidSlot, Supplier<Rotation2d> targetSetPoint, ControlState controlState) {}

