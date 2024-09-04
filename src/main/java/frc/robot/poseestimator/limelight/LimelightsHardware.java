package frc.robot.poseestimator.limelight;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import frc.utils.GBSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class LimelightsHardware extends GBSubsystem {

    private List<Limelight> limelights;


    public LimelightsHardware(String[] limelightNames, String limelightManagerName) {
        super(limelightManagerName + "Hardware/");

        limelights = new ArrayList<>();
        for (String limelightName : limelightNames) {
            limelights.add(new Limelight(limelightName));
        }
    }

    public List<Limelight> getAllLimelights() {
        return limelights;
    }

    @Override
    protected void subsystemPeriodic() {

    }
}
