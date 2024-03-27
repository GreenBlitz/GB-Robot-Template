package frc.robot.subsystems.swerve.mk4iswerve.mk4imodules;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.ModuleConstants;
import frc.robot.subsystems.swerve.ModuleUtils;
import frc.robot.subsystems.swerve.swerveinterface.IModule;
import frc.robot.subsystems.swerve.swerveinterface.ModuleInputsAutoLogged;
import frc.utils.Conversions;
import frc.utils.devicewrappers.GBTalonFXPro;

import java.util.Queue;

public class MK4IModule implements IModule {

    private final GBTalonFXPro steerMotor, driveMotor;
    private final CANcoder steerEncoder;
    private final MK4IData mk4IData;
    private Queue<Double> steerPositionQueue, drivePositionQueue;

    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0).withEnableFOC(MK4IModuleConstants.ENABLE_FOC);
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0).withEnableFOC(MK4IModuleConstants.ENABLE_FOC);

    public MK4IModule(ModuleUtils.ModuleName moduleName) {
        MK4IModuleConfigObject moduleConfigObject = getModuleConfigObject(moduleName);
        this.steerEncoder = moduleConfigObject.getSteerEncoder();
        this.driveMotor = moduleConfigObject.getDriveMotor();
        this.steerMotor = moduleConfigObject.getSteerMotor();
        this.mk4IData = new MK4IData(moduleConfigObject);

        steerMotor.setPosition(0);
//        this.steerPositionQueue = TalonFXOdometryThread6328.getInstance().registerSignal(steerMotor, moduleConfigObject.steerPositionSignal);
//        this.drivePositionQueue = TalonFXOdometryThread6328.getInstance().registerSignal(driveMotor, moduleConfigObject.drivePositionSignal);
    }

    public MK4IModuleConfigObject getModuleConfigObject(ModuleUtils.ModuleName moduleName){
        return switch (moduleName) {
            case FRONT_LEFT -> MK4IModuleConstants.FRONT_LEFT;
            case FRONT_RIGHT -> MK4IModuleConstants.FRONT_RIGHT;
            case BACK_LEFT -> MK4IModuleConstants.BACK_LEFT;
            case BACK_RIGHT -> MK4IModuleConstants.BACK_RIGHT;
        };
    }

    @Override
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

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        final double optimizedVelocityRevolutionsPerSecond = ModuleUtils.removeCouplingFromRevolutions(
                targetVelocityMetersPerSecond,
                mk4IData.getSteerMotorVelocity(),
                MK4IModuleConstants.COUPLING_RATIO
        );
        driveMotor.setControl(driveVelocityRequest.withVelocity(optimizedVelocityRevolutionsPerSecond));
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }
    
    @Override
    public void resetByEncoder() {
        steerMotor.setPosition(mk4IData.getEncoderAbsolutePosition().getRotations());
    }
    
    @Override
    public void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    @Override
    public void setBrake(boolean brake) {
        final NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.setNeutralMode(neutralModeValue);
        steerMotor.setNeutralMode(neutralModeValue);
    }

    @Override
    public void updateInputs(ModuleInputsAutoLogged inputs) {
        inputs.steerEncoderAngleDegrees = mk4IData.getEncoderAbsolutePosition().getDegrees();
        inputs.steerEncoderVelocity = steerEncoder.getVelocity().getValue();
        inputs.steerEncoderVoltage = steerEncoder.getSupplyVoltage().getValue();

        inputs.steerAngleDegrees = mk4IData.getSteerMotorPosition().getDegrees();
//        inputs.odometryUpdatesSteerAngleDegrees = steerPositionQueue.stream().mapToDouble(Conversions::revolutionsToDegrees).toArray();
        inputs.steerVoltage = mk4IData.getSteerVoltageSignal().getValue();
        inputs.steerVelocity = mk4IData.getSteerMotorVelocity().getRotations();

        inputs.driveDistanceMeters = mk4IData.getDriveMotorPositionInMeters();
//        inputs.odometryUpdatesDriveDistanceMeters = drivePositionQueue.stream().mapToDouble(this::toDriveDistance).toArray();
        inputs.driveVelocityMetersPerSecond = mk4IData.getDriveMotorVelocityInMeters();
        inputs.driveCurrent = mk4IData.getDriveStatorCurrentSignal().getValue();
        inputs.driveVoltage = mk4IData.getDriveVoltageSignal().getValue();

//        steerPositionQueue.clear();
//        drivePositionQueue.clear();
    }

}
