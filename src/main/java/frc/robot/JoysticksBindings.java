package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.constants.Phoenix6Constants;
import frc.robot.constants.Ports;
import frc.utils.GBSubsystem;
import frc.utils.calibration.staticcharacterization.StaticCharacterizationObject;
import frc.utils.devicewrappers.TalonFXWrapper;
import frc.utils.joysticks.SmartJoystick;
import frc.utils.utilcommands.LoggedDashboardCommand;

public class JoysticksBindings {

    private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.MAIN);

    private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.SECOND);

    private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.THIRD);

    private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.FOURTH);

    static TalonFXWrapper motor = new TalonFXWrapper(0, Phoenix6Constants.CANIVORE_NAME);
    static {
        motor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(150 / 7.0));
    }
    static StaticCharacterizationObject staticCharacterizationObject = new StaticCharacterizationObject(
            new GBSubsystem() {
                @Override
                protected String getLogPath() {
                    return "";
                }

                @Override
                protected void subsystemPeriodic() {

                }
            }, volt -> motor.setControl(new VoltageOut(volt)), () -> motor.getLatencyCompensatedVelocity()
    );

    public static void configureBindings() {
        mainJoystickButtons();
        secondJoystickButtons();
        thirdJoystickButtons();
        fourthJoystickButtons();
    }

    private static void mainJoystickButtons() {
        SmartJoystick usedJoystick = MAIN_JOYSTICK;
        // bindings
        usedJoystick.A.onTrue(staticCharacterizationObject.getFindKsKgCommand());
        usedJoystick.B.onTrue(new LoggedDashboardCommand("hey", volt -> motor.setControl(new VoltageOut(volt))));
    }

    private static void secondJoystickButtons() {
        SmartJoystick usedJoystick = SECOND_JOYSTICK;
        // bindings
    }

    private static void thirdJoystickButtons() {
        SmartJoystick usedJoystick = THIRD_JOYSTICK;
        // bindings
    }

    private static void fourthJoystickButtons() {
        SmartJoystick usedJoystick = FOURTH_JOYSTICK;
        // bindings
    }

}
