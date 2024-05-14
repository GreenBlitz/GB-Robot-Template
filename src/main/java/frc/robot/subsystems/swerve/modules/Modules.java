package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.swervestatehelpers.LoopMode;
import frc.utils.mirrorutils.MirrorableRotation2d;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.Arrays;

public class Modules {

    private final Module[] modules = getModules();

    private Module[] getModules() {
        return new Module[]{
                new Module(ModuleUtils.ModuleName.FRONT_LEFT),
                new Module(ModuleUtils.ModuleName.FRONT_RIGHT),
                new Module(ModuleUtils.ModuleName.BACK_LEFT),
                new Module(ModuleUtils.ModuleName.BACK_RIGHT),
        };
    }

    public void periodic() {
        for (Module currentModule : modules) {
            currentModule.periodic();
        }
    }

    public void stop() {
        for (Module currentModule : modules) {
            currentModule.stop();
        }
    }

    public void setBrake(boolean brake) {
        for (Module currentModule : modules) {
            currentModule.setBrake(brake);
        }
    }

    public void resetModulesAngleByEncoder() {
        for (Module module : getModules()) {
            module.resetByEncoder();
        }
    }

    public void setClosedLoopForModules(LoopMode loopMode) {
        for (Module currentModule : modules) {
            currentModule.setDriveMotorClosedLoop(loopMode.isClosedLoop);
        }
    }


    public Rotation2d[] getModulesDriveDistances() {
        return Arrays.stream(modules).map(Module::getDriveDistanceAngle).toArray(Rotation2d[]::new);
    }


    /**
     * Point all wheels in same angle
     *
     * @param targetAngle - angle to point to
     */
    public void pointWheels(MirrorableRotation2d targetAngle) {
        Rotation2d targetMirroredAngle = targetAngle.get();
        for (Module module : modules) {
            module.setTargetState(new SwerveModuleState(0, targetMirroredAngle));
        }
    }

    /**
     * Lock swerve wheels in X position, so it's hard to move it.
     */
    public void lockSwerve() {
        SwerveModuleState frontLeftBackRight = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        SwerveModuleState frontRightBackLeft = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));

        modules[0].setTargetState(frontLeftBackRight);
        modules[1].setTargetState(frontRightBackLeft);
        modules[2].setTargetState(frontRightBackLeft);
        modules[3].setTargetState(frontLeftBackRight);
    }


    public boolean isModulesAtStates() {
        boolean isAtStates = true;
        for (Module module : modules) {
            isAtStates = isAtStates && module.isAtTargetState();
        }
        return isAtStates;
    }

    public void setTargetModuleStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(swerveModuleStates[i]);
        }
    }


    public SwerveDriveWheelPositions getSwerveWheelPositions(int odometryUpdateIndex) {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            swerveModulePositions[i] = modules[i].getOdometryPosition(odometryUpdateIndex);
        }
        return new SwerveDriveWheelPositions(swerveModulePositions);
    }


    @AutoLogOutput(key = SwerveConstants.SWERVE_LOG_PATH + "CurrentModulesStates")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getCurrentState();
        }

        return states;
    }

    @AutoLogOutput(key = SwerveConstants.SWERVE_LOG_PATH + "TargetModulesStates")
    public SwerveModuleState[] getTargetStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getTargetState();
        }

        return states;
    }

}
