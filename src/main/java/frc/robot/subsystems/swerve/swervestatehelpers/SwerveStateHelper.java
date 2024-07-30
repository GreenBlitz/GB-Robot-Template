package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

import static frc.robot.subsystems.swerve.swervestatehelpers.AimAssistUtils.getRotationAssistedSpeeds;

public class SwerveStateHelper {

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Swerve swerve;
    private final SwerveConstants swerveConstants;
    private final Supplier<Optional<Translation2d>> noteTranslationSupplier;

    public SwerveStateHelper(Supplier<Pose2d> robotPoseSupplier, Supplier<Optional<Translation2d>> noteTranslationSupplier,
            Swerve swerve) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.swerve = swerve;
        this.swerveConstants = swerve.getConstants();
        this.noteTranslationSupplier = noteTranslationSupplier;
    }

    public Translation2d getRotationAxis(RotateAxis rotationAxisState) {
        return switch (rotationAxisState) {
            case FRONT_LEFT_MODULE -> swerveConstants.LOCATIONS[ModuleUtils.ModuleName.FRONT_LEFT.getIndex()];
            case FRONT_RIGHT_MODULE -> swerveConstants.LOCATIONS[ModuleUtils.ModuleName.FRONT_RIGHT.getIndex()];
            case BACK_LEFT_MODULE -> swerveConstants.LOCATIONS[ModuleUtils.ModuleName.BACK_LEFT.getIndex()];
            case BACK_RIGHT_MODULE -> swerveConstants.LOCATIONS[ModuleUtils.ModuleName.BACK_RIGHT.getIndex()];
            case MIDDLE_OF_ROBOT -> new Translation2d();
        };
    }

    public ChassisSpeeds applyAimAssistOnInputsSpeeds(AimAssist aimAssistState, ChassisSpeeds inputSpeeds) {
        return switch (aimAssistState) {
            case SPEAKER -> handleSpeakerState(
                    inputSpeeds,
                    robotPoseSupplier,
                    robotPose -> FieldConstants.getSpeaker().toTranslation2d().minus(robotPose.getTranslation()).getAngle(),
                    swerveConstants
            );
            case AMP -> handleAmpState(
                    inputSpeeds,
                    robotPoseSupplier,
                    (robotPose -> Rotation2d.fromDegrees(90)),
                    swerveConstants
            );
            case NOTE -> handleNoteAssistState(
                    inputSpeeds,
                    robotPoseSupplier,
                    noteTranslationSupplier
            );
            case NONE -> inputSpeeds;
            default -> inputSpeeds;
        };
    }


    private ChassisSpeeds handleNoteAssistState(ChassisSpeeds inputSpeeds, Supplier<Pose2d> robotPoseSupplier,
            Supplier<Optional<Translation2d>> noteTranslationSupplier) {
        if (noteTranslationSupplier.get().isEmpty()) {
            return inputSpeeds;
        }
        Rotation2d robotRotation = robotPoseSupplier.get().getRotation().unaryMinus();

        Translation2d noteTranslation = noteTranslationSupplier.get().get();
        Translation2d normalizedNoteTranslation = noteTranslation.minus(robotPoseSupplier.get().getTranslation());
        Translation2d rotatedNoteTranslation = normalizedNoteTranslation.rotateBy(robotRotation);

        Logger.recordOutput("note pose", new Pose2d(noteTranslation, new Rotation2d()));

        double wantedHorizontalVelocity = swerveConstants.translationMetersPIDController().calculate(0, rotatedNoteTranslation.getY())
                + inputSpeeds.vyMetersPerSecond;
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                inputSpeeds.vxMetersPerSecond,
                wantedHorizontalVelocity,
                inputSpeeds.omegaRadiansPerSecond,
                robotRotation.unaryMinus()
        );
    }

    private ChassisSpeeds handleAmpState(ChassisSpeeds inputSpeeds, Supplier<Pose2d> robotPoseSupplier,
            Function<Pose2d, Rotation2d> targetRotationSupplier, SwerveConstants swerveConstants) {
        return getRotationAssistedSpeeds(
                inputSpeeds,
                swerve.getRobotRelativeVelocity(),
                robotPoseSupplier,
                targetRotationSupplier,
                swerveConstants
        );
    }

    private ChassisSpeeds handleSpeakerState(ChassisSpeeds inputSpeeds,
            Supplier<Pose2d> robotPoseSupplier,
            Function<Pose2d, Rotation2d> targetRotationSupplier, SwerveConstants swerveConstants) {
        return getRotationAssistedSpeeds(
                inputSpeeds,
                swerve.getRobotRelativeVelocity(),
                robotPoseSupplier,
                targetRotationSupplier,
                swerveConstants
        );
    }

}
