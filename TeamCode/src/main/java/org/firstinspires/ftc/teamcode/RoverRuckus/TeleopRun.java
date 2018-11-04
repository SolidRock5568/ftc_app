package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleopRun", group="RoverRuckus")
//@Disabled
public class TeleopRun extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    RobotConfig robot = new RobotConfig();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    TelemetryPacket.Adapter dashboardTelemetry = new TelemetryPacket.Adapter(dashboard);

    double LeftJoystickY = 0;
    double LeftJoystickX = 0;
    double RightJoystickX = 0;

    double LeftEncoder = 0;
    double RightEncoder = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        robot.InitServos();
        robot.KillMotors();
        //robot.VuforiaInit();
        //robot.UpdateTelemetry();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        //robot.UpdateTelemetry();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //dashboard.setTelemetryTransmissionInterval(100);
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
//
//        robot.FancySwerve(LeftJoystickY, LeftJoystickX, RightJoystickX);
//        robot.SwerveDrive(-LeftJoystickY, LeftJoystickX, RightJoystickX);
//
        if (gamepad1.a)
        {
            robot.FrontLeftServo.setPosition(gamepad1.left_stick_x*.6);
            robot.FrontRightServo.setPosition(gamepad1.left_stick_x*.6);
            robot.BackLeftServo.setPosition(gamepad1.left_stick_x*.6);
            robot.BackRightServo.setPosition(gamepad1.left_stick_x*.6);
        }
        else if(!gamepad1.a)
        {
            robot.FrontLeftServo.setPosition(0);
            robot.FrontRightServo.setPosition(0);
            robot.BackLeftServo.setPosition(0);
            robot.BackRightServo.setPosition(0);
        }

        if (!gamepad2.a) {
            robot.SetLiftMotors(gamepad2.left_stick_y *.5);
        }
        else {
            robot.SetLiftMotors(gamepad2.left_stick_y);
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
        dashboardTelemetry.addData("Encoder", robot.LiftMotorOne.getCurrentPosition());
        dashboardTelemetry.addData("Gamepad 1 A", gamepad1.a);
//        packet.put("PacketAdapter", dashboardTelemetry);
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
