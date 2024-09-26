package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.RobotType;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;

public class PivotFactory {

    public static Pivot generatePivot(String logPath) {
        return switch (Robot.ROBOT_TYPE) {
            case REAL -> getRealPivot(logPath);
            default -> null;
        };
    }

    private static Pivot getRealPivot(String logPath) {
        TalonFXWrapper motorWrapper = new TalonFXWrapper(IDs.TalonFXs.PIVOT_MOTOR_ID);
        motorWrapper.applyConfiguration(PivotRealConstants.TALON_FX_CONFIGURATION);

        return new Pivot(
                logPath,
                PivotRealConstants.getTalonFXMotor(logPath, motorWrapper),
                new Phoenix6AngleRequest(new PositionVoltage(0).withEnableFOC(true)),
                Phoenix6SignalBuilder.generatePhoenix6Signal(motorWrapper.getPosition(), 100, AngleUnit.ROTATIONS),
                Phoenix6SignalBuilder.generatePhoenix6Signal(motorWrapper.getPosition(), GlobalConstants.CANIVORE_UPDATE_FREQUENCY),
                Phoenix6SignalBuilder.generatePhoenix6Signal(motorWrapper.getVelocity(), GlobalConstants.CANIVORE_UPDATE_FREQUENCY),
                Phoenix6SignalBuilder.generatePhoenix6Signal(motorWrapper.getMotorVoltage(), GlobalConstants.CANIVORE_UPDATE_FREQUENCY),
                Phoenix6SignalBuilder.generatePhoenix6Signal(motorWrapper.getStatorCurrent(), GlobalConstants.CANIVORE_UPDATE_FREQUENCY)
        );
    }

    public static Rotation2d getTolerance(){
        return switch (Robot.ROBOT_TYPE){
            case REAL -> PivotRealConstants.TOLERANCE;
            default -> new Rotation2d();
        };
    }

}
