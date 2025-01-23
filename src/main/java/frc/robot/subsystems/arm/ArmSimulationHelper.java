package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ArmSimulationHelper {

    public static Pose3d getArmSimulationPose(Rotation2d armAngle, double elevatorHeightMeters){
        return new Pose3d(
                new Translation3d(0,0, elevatorHeightMeters + ElevatorConstants.STAGE_HEIGHT),
                new Rotation3d(0,armAngle.unaryMinus().getRadians(),0)
        );
    }

}
