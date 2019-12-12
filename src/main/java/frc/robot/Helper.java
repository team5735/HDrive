package frc.robot;

public class Helper {
    public static double round(double num, int round) {
        double val = num*(10*round);
        val = Math.round(val);
        val = val /(10*round);
        return val;
    }

    public static double deadband(double input, double deadband) {
        if(Math.abs(input) < deadband) return 0;
        return input;
    }
}