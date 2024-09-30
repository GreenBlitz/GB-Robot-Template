package frc.robot.subsystems.lifter.factory;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.request.phoenix6.Phoenix6DoubleRequest;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.robot.subsystems.lifter.Lifter;
import frc.robot.subsystems.lifter.LifterStuff;
import frc.utils.AngleUnit;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class LifterRealConstants {

    private static SysIdRoutine.Config generateSysidConfig() {
        return new SysIdRoutine.Config(
                Volts.of(1).per(Seconds.of(1)),
                Volts.of(7),
                Seconds.of(10),
                (state) -> SignalLogger.writeString("state", state.toString())
        );
    }

    public static LifterStuff generateLifterStuff(String logPath) {

        TalonFXWrapper talonFXWrapper = new TalonFXWrapper(IDs.TalonFXIDs.LIFTER);

        return new LifterStuff(
                logPath,
                new TalonFXMotor(logPath, talonFXWrapper, generateSysidConfig()),
                Phoenix6SignalBuilder.generatePhoenix6Signal(talonFXWrapper.getPosition(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS),
                new Phoenix6DoubleRequest(new VoltageOut(0))
        );
    }


}
