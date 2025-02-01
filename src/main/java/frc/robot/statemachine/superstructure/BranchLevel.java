package frc.robot.statemachine.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;

public enum BranchLevel {

    L2(ElevatorState.L2.getHeightMeters(), ArmState.L2.getPosition()),
    L3(ElevatorState.L3.getHeightMeters(), ArmState.L3.getPosition()),
    L4(ElevatorState.L4.getHeightMeters(), ArmState.L4.getPosition());

    private final double elevatorPositionMeters;
    private final Rotation2d armPosition;

    BranchLevel(double elevatorPositionMeters, Rotation2d armPosition) {
        this.elevatorPositionMeters = elevatorPositionMeters;
        this.armPosition = armPosition;
    }

    public double getElevatorPositionMeters() {
        return elevatorPositionMeters;
    }

    public Rotation2d getArmPosition() {
        return armPosition;
    }

}