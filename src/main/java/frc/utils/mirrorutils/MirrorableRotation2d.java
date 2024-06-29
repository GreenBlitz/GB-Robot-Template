package frc.utils.mirrorutils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.MathConstants;

public class MirrorableRotation2d extends Mirrorable<Rotation2d> {

    public MirrorableRotation2d(Rotation2d nonMirroredRotation, boolean mirrorWhenNotOnRelativeAlliance) {
        super(nonMirroredRotation, mirrorWhenNotOnRelativeAlliance);
    }

    public MirrorableRotation2d(double radians, boolean mirrorWhenNotOnRelativeAlliance) {
        this(new Rotation2d(radians), mirrorWhenNotOnRelativeAlliance);
    }

    public static MirrorableRotation2d fromDegrees(double degrees, boolean mirrorWhenNotOnRelativeAlliance) {
        return new MirrorableRotation2d(Rotation2d.fromDegrees(degrees), mirrorWhenNotOnRelativeAlliance);
    }

    public static MirrorableRotation2d fromRadians(double radians, boolean mirrorWhenNotOnRelativeAlliance) {
        return new MirrorableRotation2d(Rotation2d.fromRadians(radians), mirrorWhenNotOnRelativeAlliance);
    }

    public static MirrorableRotation2d fromRotations(double rotations, boolean mirrorWhenNotOnRelativeAlliance) {
        return new MirrorableRotation2d(Rotation2d.fromRotations(rotations), mirrorWhenNotOnRelativeAlliance);
    }

    @Override
    protected Rotation2d mirror(Rotation2d rotation) {
        return MathConstants.HALF_CIRCLE.minus(rotation);
    }

}
