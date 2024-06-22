package frc.utils.mirrorutils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.MathConstants;

public class MirrorableRotation2d extends Mirrorable<Rotation2d> {

    public MirrorableRotation2d(Rotation2d nonMirroredRotation, boolean mirrorWhenRedAlliance) {
        super(nonMirroredRotation, mirrorWhenRedAlliance);
    }

    public MirrorableRotation2d(double radians, boolean mirrorWhenRedAlliance) {
        this(new Rotation2d(radians), mirrorWhenRedAlliance);
    }

    public static MirrorableRotation2d fromDegrees(double degrees, boolean mirrorWhenRedAlliance) {
        return new MirrorableRotation2d(Rotation2d.fromDegrees(degrees), mirrorWhenRedAlliance);
    }

    public static MirrorableRotation2d fromRadians(double radians, boolean mirrorWhenRedAlliance) {
        return new MirrorableRotation2d(Rotation2d.fromRadians(radians), mirrorWhenRedAlliance);
    }

    public static MirrorableRotation2d fromRotations(double rotations, boolean mirrorWhenRedAlliance) {
        return new MirrorableRotation2d(Rotation2d.fromRotations(rotations), mirrorWhenRedAlliance);
    }

    @Override
    protected Rotation2d mirror(Rotation2d rotation) {
        return MathConstants.HALF_CIRCLE.minus(rotation);
    }

}
