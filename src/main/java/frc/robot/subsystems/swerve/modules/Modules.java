package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.MathConstants;
import frc.robot.subsystems.swerve.swervestatehelpers.LoopMode;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.Arrays;

public class Modules {

    private final Module[] modules;

    public Modules(Module[] modules){
        this.modules = modules;
    }

    public void logStatus(){
        for (Module currentModule : modules) {
            currentModule.logStatus();
        }
    }


    public void resetModulesAngleByEncoder() {
        for (Module module : modules) {
            module.resetByEncoder();
        }
    }

    public void setClosedLoopForModules(LoopMode loopMode) {
        for (Module currentModule : modules) {
            currentModule.setClosedLoop(loopMode.isClosedLoop);
        }
    }

    public void setBrake(boolean brake) {
        for (Module currentModule : modules) {
            currentModule.setBrake(brake);
        }
    }


    public void pointWheels(Rotation2d targetAngle, boolean optimize) {
        for (Module module : modules) {
            module.pointToAngle(targetAngle, optimize);
        }
    }

    public void pointWheelsInCircle() {
        boolean optimizeAngle = true;
        modules[0].pointToAngle(MathConstants.EIGHTH_CIRCLE.unaryMinus(), optimizeAngle);
        modules[1].pointToAngle(MathConstants.EIGHTH_CIRCLE, optimizeAngle);
        modules[2].pointToAngle(MathConstants.EIGHTH_CIRCLE, optimizeAngle);
        modules[3].pointToAngle(MathConstants.EIGHTH_CIRCLE.unaryMinus(), optimizeAngle);
    }

    public void pointWheelsInX() {
        SwerveModuleState frontLeftBackRight = new SwerveModuleState(0, MathConstants.EIGHTH_CIRCLE);
        SwerveModuleState frontRightBackLeft = new SwerveModuleState(0, MathConstants.EIGHTH_CIRCLE.unaryMinus());

        modules[0].setTargetState(frontLeftBackRight);
        modules[1].setTargetState(frontRightBackLeft);
        modules[2].setTargetState(frontRightBackLeft);
        modules[3].setTargetState(frontLeftBackRight);
    }


    /**
     * Runs swerve module around itself for Sysid Steer Calibration
     *
     * @param voltage - voltage to run the swerve module steer
     */
    public void runModuleSteerByVoltage(ModuleUtils.ModuleName module, double voltage) {
        modules[module.getIndex()].runSteerMotorByVoltage(voltage);
    }

    /**
     * Runs swerve module around itself for Sysid Steer Calibration
     *
     * @param voltage - voltage to run the swerve module drive
     */
    public void runModulesDriveByVoltage(double voltage) {
        for (Module module : modules) {
            module.runDriveMotorByVoltage(voltage);
        }
    }


    public void setTargetModuleStates(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(moduleStates[i]);
        }
    }

    public void stop() {
        for (Module currentModule : modules) {
            currentModule.stop();
        }
    }


    public boolean isModulesAtVelocities() {
        for (Module module : modules) {
            if (!module.isAtTargetVelocity()) {
                return false;
            }
        }
        return true;
    }

    public boolean isModulesAtAngles() {
        for (Module module : modules) {
            if (!module.isAtTargetAngle()) {
                return false;
            }
        }
        return true;
    }

    public boolean isModulesAtStates() {
        return isModulesAtAngles() && isModulesAtAngles();
    }


    @AutoLogOutput(key = ModuleConstants.LOG_PATH + "TargetModulesStates")
    public SwerveModuleState[] getTargetStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getTargetState();
        }

        return states;
    }

    @AutoLogOutput(key = ModuleConstants.LOG_PATH + "CurrentModulesStates")
    public SwerveModuleState[] getModulesStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getCurrentState();
        }

        return states;
    }


    public Rotation2d[] getModulesDriveDistances() {
        return Arrays.stream(modules).map(Module::getDriveDistanceAngle).toArray(Rotation2d[]::new);
    }

    public SwerveDriveWheelPositions getSwerveWheelPositions(int odometrySampleIndex) {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            swerveModulePositions[i] = modules[i].getOdometryPosition(odometrySampleIndex);
        }
        return new SwerveDriveWheelPositions(swerveModulePositions);
    }

    public SwerveDriveWheelPositions[] getAllSwerveWheelPositionSamples() {
        int numberOfOdometrySamples = modules[0].getNumberOfOdometrySamples();
        SwerveDriveWheelPositions[] swerveWheelPositions = new SwerveDriveWheelPositions[numberOfOdometrySamples];
        for (int i = 0; i < numberOfOdometrySamples; i++) {
            swerveWheelPositions[i] = getSwerveWheelPositions(i);
        }
        return swerveWheelPositions;
    }

}
