package frc.utils.ObjectVisualizer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Pose3dGB extends Pose3d{
    public Pose3dGB(Rotation3d rotation3d, double... components){
        super(components[0],components[1],components[2],rotation3d);
    }

}
