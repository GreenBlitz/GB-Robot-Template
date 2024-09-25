package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class PivotConstants {

    public static Pivot getPivot (String logPath){

        TalonFXWrapper motorWrapper = new TalonFXWrapper(IDs.TalonFXs.PIVOT_MOTOR_ID);

        return new Pivot(
                logPath,
                new TalonFXMotor(
                        logPath,
                        motorWrapper,
                        new SysIdRoutine.Config(
                                Volts.of(1).per(Seconds.of(1)),
                                Volts.of(7),
                                Seconds.of(10),
                                (state) -> SignalLogger.writeString("state", state.toString())
                        )
                ),
                new Phoenix6AngleRequest(new PositionVoltage(0)),
                Phoenix6SignalBuilder.generatePhoenix6Signal(motorWrapper.getPosition(), 100, AngleUnit.ROTATIONS)
        );
    }
}
