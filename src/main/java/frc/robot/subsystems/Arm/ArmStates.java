package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmStates {
    INTAKE(Rotation2d.fromDegrees(-115)),
    LOW_DROP(Rotation2d.fromDegrees(-10)),
    MID_DROP(Rotation2d.fromDegrees(65)),
    HIGH_DROP(Rotation2d.fromDegrees(125)),
    SAFE_HOLD(Rotation2d.fromDegrees(-80));


    private final Rotation2d position;

    ArmStates(Rotation2d position){
        this.position = position;
    }

    public Rotation2d getPosition(){
        return position;
    }
}
