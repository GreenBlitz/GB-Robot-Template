package frc.robot.subsystems.swerve.swervestatehelpers;

import frc.utils.allianceutils.AllianceRotation2d;
import frc.utils.allianceutils.AllianceTranslation2d;

import java.util.function.Supplier;

public enum AimAssist {

    NOTE(),
    SPEAKER(),
    AMP(),
    STAGE();

    public final Supplier<AllianceRotation2d> targetAngleSupplier;

    AimAssist(AllianceRotation2d targetAllianceRotation) {}

    AimAssist(AllianceRotation2dSupplier targetAllianceRotation) {}

    AimAssist(AllianceTranslation2dSupplier targetAllianceTranslation) {}

    private interface AllianceRotation2dSupplier extends Supplier<AllianceRotation2d> {}

    private interface AllianceTranslation2dSupplier extends Supplier<AllianceTranslation2d> {}

    //Todo - pose to angle. Maybe in kinda math util or pose util
}
