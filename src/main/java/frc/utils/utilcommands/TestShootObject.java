package frc.utils.utilcommands;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

//test class for this shooting
public class TestShootObject extends ShootObject {

    public TestShootObject() {
        super(5, 10, () -> Rotation2d.fromDegrees(45), new Translation3d(), () -> new Pose3d(5,5,5, new Rotation3d()), 10);
    }
}
