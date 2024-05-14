package frc.robot.constants;

import frc.utils.mirrorutils.MirrorableRotation2d;
import frc.utils.mirrorutils.MirrorableTranslation3d;

public class FieldConstants {

    public static final double FIELD_LENGTH_METERS = 16.54175;

    public static final double FIELD_WIDTH_METERS = 8.0137;

    public static final MirrorableTranslation3d SPEAKER = new MirrorableTranslation3d(
            0.23,
            FIELD_WIDTH_METERS - 2.55,
            2.045,
            true
    );

    public static final MirrorableRotation2d ANGLE_TO_AMP = MirrorableRotation2d.fromDegrees(90, false);

}
