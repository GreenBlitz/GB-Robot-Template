package frc.utils.mirrorutils;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.FieldConstants;

public class MirrorableTranslation3d extends Mirrorable<Translation3d> {

    public MirrorableTranslation3d(Translation3d nonMirroredTranslation, boolean mirrorWhenNotOnRelativeAlliance) {
        super(nonMirroredTranslation, mirrorWhenNotOnRelativeAlliance);
    }

    public MirrorableTranslation3d(double x, double y, double z, boolean mirrorWhenNotOnRelativeAlliance) {
        this(new Translation3d(x, y, z), mirrorWhenNotOnRelativeAlliance);
    }

    @Override
    protected Translation3d mirror(Translation3d translation) {
        return new Translation3d(
                FieldConstants.FIELD_LENGTH_METERS - translation.getX(),
                translation.getY(),
                translation.getZ()
        );
    }

}
