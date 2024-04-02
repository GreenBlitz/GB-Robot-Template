package frc.robot.commands.calibration;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.commands.GBCommand;
import frc.utils.devicewrappers.GBTalonFXPro;
import org.littletonrobotics.junction.Logger;

public class FindKS extends GBCommand {

    private static final double VOLTAGE_RAMP = 0.01;
    private static final double STARTED_MOVING_SPEED = 0.001;
    private static final double TIME_BETWEEN_VOLTAGE_RAMPS_SECONDS = 0.1;

    private final Timer timer;

    private double lastTime;
    private double lastVoltage;
    private final double kg;

    private final GBTalonFXPro motor;
    private final VoltageOut voltageOutControl;

    //Todo - add motor sub sys with SYSID and the functions: getVelocity, setVoltage
    //Todo - instead of getting motor and control get motorSubSys

    public FindKS(double kg, GBTalonFXPro motor, VoltageOut voltageOutControl) {
        this.timer = new Timer();
        this.kg = kg;
        this.motor = motor;
        this.voltageOutControl = voltageOutControl;
        lastTime = 0;
        lastVoltage = 0;
    }

    @Override
    public void initialize() {
        timer.restart();
        lastTime = timer.get();
    }

    @Override
    public void execute() {
        if (timer.get() - lastTime >= TIME_BETWEEN_VOLTAGE_RAMPS_SECONDS) {
            lastVoltage += VOLTAGE_RAMP;
            motor.setControl(voltageOutControl.withOutput(lastVoltage + kg));
            lastTime = timer.get();
        }
    }

    @Override
    public boolean isFinished() {
        return motor.getVelocity().getValue() >= STARTED_MOVING_SPEED;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("KS OF MOTOR: " + motor.getDeviceID(), lastVoltage);
    }
}
