package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DashboardVariables {
    public static double VirtualLeftJoystickY = 0;
    public static double VirtualLeftJoystickX = 0;
    public static double VirtualRightJoystickX = 0;
    public static boolean VirtualJoystick = true;
    public static boolean FancySwerve = true;

    public static void setFancySwerve(boolean fancySwerve) {
        FancySwerve = fancySwerve;
    }

    public static boolean isFancySwerve() {
        return FancySwerve;
    }

    public static void setVirtualLeftJoystickY(double virtualLeftJoystickY) {
        VirtualLeftJoystickY = virtualLeftJoystickY;
    }

    public static double getVirtualLeftJoystickY() {
        return VirtualLeftJoystickY;
    }

    public static void setVirtualRightJoystickX(double virtualRightJoystickX) {
        VirtualRightJoystickX = virtualRightJoystickX;
    }

    public static double getVirtualRightJoystickX() {
        return VirtualRightJoystickX;
    }

    public static void setVirtualLeftJoystickX(double virtualLeftJoystickX) {
        VirtualLeftJoystickX = virtualLeftJoystickX;
    }

    public static double getVirtualLeftJoystickX() {
        return VirtualLeftJoystickX;
    }

    public static void setVirtualJoystick(boolean virtualJoystick) {
        VirtualJoystick = virtualJoystick;
    }

    public static boolean isVirtualJoystick() {
        return VirtualJoystick;
    }
}
