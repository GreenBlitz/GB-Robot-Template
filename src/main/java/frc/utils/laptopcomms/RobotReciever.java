package frc.utils.laptopcomms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.GBSubsystem;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.Socket;

public class RobotReciever extends GBSubsystem {

    private Socket socket;
    private BufferedReader input;

    private String lastMessage;

    public RobotReciever() {
        lastMessage = "";
        try {
            socket = new Socket(LaptopSender.ROBOT_IP, LaptopSender.PORT);
            input = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            SmartDashboard.putBoolean("isRecieverWorking", true);
        } catch (Exception e) {
            SmartDashboard.putBoolean("isRecieverWorking", false);

        }
    }

    @Override
    public void periodic() {
        super.periodic();
        recieveMessage();
        SmartDashboard.putString("MESSAGE: ",lastMessage);
    }

    private void recieveMessage() {
        try {
            lastMessage = input.readLine();
            System.out.println(lastMessage);
        } catch (Exception e) {

        }
    }

}
