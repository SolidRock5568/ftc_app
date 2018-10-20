package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="TeleopRun", group="RoverRuckus")
//@Disabled
public class TeleopRun extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public RobotConfig robot = new RobotConfig();

    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();
    public TelemetryPacket.Adapter dashboardTelemetry = new TelemetryPacket.Adapter(dashboard);

    public double LeftJoystickY = 0;
    public double LeftJoystickX = 0;
    public double RightJoystickX = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        robot.InitServos();
        robot.KillMotors();
        robot.VuforiaInit();
        robot.UpdateTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot.UpdateTelemetry();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //gamepad1.left_stick_y is the forward and back joystick, but inverted
        //gamepad1.left_stick_x is the strafe left and right joystick
        //gamepad1.right_stick_x is the turn left and turn right joystick

        LeftJoystickX = DashboardVariables.isVirtualJoystick() ? DashboardVariables.getVirtualLeftJoystickX() : gamepad1.left_stick_x;
        LeftJoystickY = DashboardVariables.isVirtualJoystick() ? DashboardVariables.getVirtualLeftJoystickY() : gamepad1.left_stick_y;
        RightJoystickX = DashboardVariables.isVirtualJoystick() ? DashboardVariables.getVirtualRightJoystickX() : gamepad1.right_stick_x;

        if (DashboardVariables.isFancySwerve()) {
            robot.FancySwerve(LeftJoystickY, LeftJoystickX, RightJoystickX);
        } else {
            robot.SwerveDrive(LeftJoystickY, LeftJoystickX, RightJoystickX);
        }

        /**
         * Dashboard Values
         */
        dashboardTelemetry.addData("Joystick: ", LeftJoystickX);
        dashboardTelemetry.addData("Scaled Angle: ", robot.ScaleValue(LeftJoystickX, -1, 1));
        dashboardTelemetry.addData("FL Unscaled Scaled Angle", robot.UnScaleValue(robot.ScaleValue(LeftJoystickX, -1, 1), robot.FrontLeftMin, robot.FrontLeftMax));
        dashboardTelemetry.addData("FR Unscaled Scaled Angle", robot.UnScaleValue(robot.ScaleValue(LeftJoystickX, -1, 1), robot.FrontRightMin, robot.FrontRightMax));
        dashboardTelemetry.addData("BL Unscaled Scaled Angle", robot.UnScaleValue(robot.ScaleValue(LeftJoystickX, -1, 1), robot.BackLeftMin, robot.BackLeftMax));
        dashboardTelemetry.addData("BR Unscaled Scaled Angle", robot.UnScaleValue(robot.ScaleValue(LeftJoystickX, -1, 1), robot.BackRightMin, robot.BackRightMax));

        packet.put("PacketAdapter", dashboardTelemetry);
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.KillMotors();
    }

}
