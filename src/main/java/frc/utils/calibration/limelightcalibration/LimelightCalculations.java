package frc.utils.calibration.limelightcalibration;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.function.Supplier;

public class LimelightCalculations {

    public static Pose3d getCameraToRobot(Supplier<Transform3d> tagToCamera, Pose3d tagToRobot){
        return tagToRobot.transformBy(tagToCamera.get().inverse());
    }
}