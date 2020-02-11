package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public class Constants {
    public static double JOYSTICK_DEADBAND = 0.1;

    public static int LEFT_MASTER_ID = 57;
    public static int LEFT_SLAVE_ID = 56;
    public static boolean LEFT_INVERTED = true;

    public static int RIGHT_MASTER_ID = 58;
    public static int RIGHT_SLAVE_ID = 59;
    public static boolean RIGHT_INVERTED = false;

    public static int CENTER_ID = 55;

    public static int HOST_TALON_ID = 6;

    public static StatorCurrentLimitConfiguration TALON_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 38.5, 38.5, 0.25);
}
