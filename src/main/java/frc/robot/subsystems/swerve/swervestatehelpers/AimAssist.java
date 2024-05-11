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

    AimAssist(RotationSupplier targetAllianceRotation) {}

    AimAssist(TranslationSupplier targetAllianceTranslation) {}

    private interface RotationSupplier extends Supplier<AllianceRotation2d> {}

    private interface TranslationSupplier extends Supplier<AllianceTranslation2d> {}

    //Todo - pose to angle. Maybe in kinda math util or pose util
}
