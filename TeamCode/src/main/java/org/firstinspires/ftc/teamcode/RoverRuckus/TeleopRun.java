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

    double BumpAmount = 0;
    double CurrentAngle= 0;

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
        BumpAmount = 0.01;
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

        //robot.FancySwerve(LeftJoystickY, LeftJoystickX, RightJoystickX);
        robot.SwerveDrive(-LeftJoystickY, LeftJoystickX, RightJoystickX);

        if (gamepad2.b) {
            robot.LiftMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.LiftMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (!gamepad2.x)
        {
            robot.SetLiftMotors(gamepad2.left_stick_y);
        }
        else
        {
            robot.SetLiftMotors(gamepad2.left_stick_y);
        }

        /*--------------------------------*/

        if (gamepad2.right_stick_y > 0.1)
        {
            robot.SetArmMotor(gamepad2.right_stick_y);
        }
        else if (gamepad2.right_stick_y < -0.1)
        {
            robot.SetArmMotor(gamepad2.right_stick_y);
        }
        else
        {
            robot.SetArmMotor(0);
        }

        if (gamepad2.left_bumper) { robot.LeftDumpServo.setPosition(0); }
        else if(!gamepad2.left_bumper) { robot.LeftDumpServo.setPosition(1); }

        if (gamepad2.right_bumper) { robot.RightDumpServo.setPosition(1); }
        else if(!gamepad2.right_bumper) { robot.RightDumpServo.setPosition(0); }

        if (gamepad2.right_trigger > 0.05)
        {
            robot.InfeedIn();
        }
        else
        {
            robot.InfeedStop();
        }



//        dashboardTelemetry.addData("Current Angle: ", CurrentAngle);
//        dashboardTelemetry.addData("Gamepad 2 dpad up: ", gamepad2.dpad_up);
//        dashboardTelemetry.addData("Bump Amount: ", BumpAmount);



//        if (gamepad2.dpad_up)
//        {
//            dashboardTelemetry.addData("Gamepad 2 pressed ", gamepad2.dpad_up);
//            robot.FrontLeftServo.setPosition(CurrentAngle);
//            robot.FrontRightServo.setPosition(CurrentAngle);
//            robot.BackLeftServo.setPosition(CurrentAngle);
//            robot.BackRightServo.setPosition(CurrentAngle);
//
//            CurrentAngle += BumpAmount;
//            try {
//                Thread.sleep(250);
//            }
//            catch (InterruptedException ex)
//            {}
//
//        }
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
        dashboardTelemetry.addData("Current Angle: ", CurrentAngle);
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
