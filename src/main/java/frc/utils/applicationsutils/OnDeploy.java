package frc.utils.applicationsutils;

import javax.swing.*;

public class OnDeploy {

    public static void main(String[] args) {
        startComputerPrograms();
    }

    private static void startComputerPrograms() {
        CMDHandler.runPythonClass("keyboard_to_nt.py");
    }

}
