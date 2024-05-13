package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.mirrorutils.MirrorablePose2d;
import frc.utils.mirrorutils.MirrorableRotation2d;

public class FieldConstants {

    public static final double FIELD_LENGTH_METERS = 16.54175;

    public static final double FIELD_WIDTH_METERS = 8.0137;

    public static final MirrorablePose2d SPEAKER = new MirrorablePose2d(0.23, FIELD_WIDTH_METERS - 2.55, new Rotation2d(), true);

    public static final MirrorableRotation2d ANGLE_TO_AMP = MirrorableRotation2d.fromDegrees(90, false);

}
