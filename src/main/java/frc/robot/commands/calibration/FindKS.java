package frc.robot.commands.calibration;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.commands.GBCommand;
import frc.utils.devicewrappers.GBTalonFXPro;
import org.littletonrobotics.junction.Logger;

public class FindKS extends GBCommand {

    private Timer timer;

    private double lastTime;
    private double lastVoltage;
    private double kg;

    private GBTalonFXPro motor;
    private VoltageOut voltageOutControl;

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
        if (timer.get() - lastTime >= 0.1) {
            lastVoltage += 0.01;
            motor.setControl(voltageOutControl.withOutput(lastVoltage + kg));
            lastTime = timer.get();
        }
    }

    @Override
    public boolean isFinished() {
        return motor.getVelocity().getValue() >= 0.001;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("KS OF MOTOR: " + motor.getDeviceID(), lastVoltage);
    }
}
