package org.firstinspires.ftc.teamcode.RoverRuckus;

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
    RobotConfig robot = new RobotConfig();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        robot.InitServos();
        robot.KillMotors();
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
        robot.SwerveDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.addData("Joystick: ", gamepad1.left_stick_x);
        telemetry.addData("Scaled Angle: ", robot.ScaleValue(gamepad1.left_stick_x, -1, 1));
        telemetry.addData("FL Unscaled Scaled Angle", robot.UnScaleValue(robot.ScaleValue(gamepad1.left_stick_x, -1, 1), robot.FrontLeftMin, robot.FrontLeftMax));

        robot.UpdateTelemetry();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.KillMotors();
    }

}
