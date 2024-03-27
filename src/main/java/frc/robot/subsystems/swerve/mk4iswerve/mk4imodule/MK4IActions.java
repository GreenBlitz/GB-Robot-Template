package frc.robot.subsystems.swerve.mk4iswerve.mk4imodule;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.ModuleConstants;
import frc.robot.subsystems.swerve.ModuleUtils;
import frc.utils.devicewrappers.GBTalonFXPro;

public class MK4IActions {

    private final GBTalonFXPro steerMotor, driveMotor;
    private final MK4IData mk4IData;


    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0).withEnableFOC(MK4IModuleConstants.ENABLE_FOC_DRIVE);
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0).withEnableFOC(MK4IModuleConstants.ENABLE_FOC_STEER);


    public MK4IActions(GBTalonFXPro driveMotor, GBTalonFXPro steerMotor, MK4IData mk4IData){
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.mk4IData = mk4IData;

        steerMotor.setPosition(0);//Todo - delete after encoder working
    }


    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        final double voltage = ModuleUtils.velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                ModuleConstants.WHEEL_DIAMETER_METERS,
                mk4IData.getSteerMotorVelocity().getRotations(),
                MK4IModuleConstants.COUPLING_RATIO,
                ModuleConstants.MAX_SPEED_REVOLUTIONS_PER_SECOND,
                ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }


    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        final double optimizedVelocityRevolutionsPerSecond = ModuleUtils.removeCouplingFromRevolutions(
                targetVelocityMetersPerSecond,
                mk4IData.getSteerMotorVelocity(),
                MK4IModuleConstants.COUPLING_RATIO
        );
        driveMotor.setControl(driveVelocityRequest.withVelocity(optimizedVelocityRevolutionsPerSecond));
    }


    public void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }


    public void resetByEncoder() {
        steerMotor.setPosition(mk4IData.getEncoderAbsolutePosition().getRotations());
    }


    public void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }


    public void setBrake(boolean brake) {
        final NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.setNeutralMode(neutralModeValue);
        steerMotor.setNeutralMode(neutralModeValue);
    }


}
