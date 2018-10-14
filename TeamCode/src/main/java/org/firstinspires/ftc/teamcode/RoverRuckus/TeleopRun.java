package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleopRun", group="RoverRuckus")
//@Disabled
public class TeleopRun extends OpMode
    {
        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        RobotConfig robot = new RobotConfig();

        /*
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void init () {
        robot.InitServos();
        robot.KillMotors();
        robot.UpdateTelemetry();
    }

        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */
        @Override
        public void init_loop () {
        robot.UpdateTelemetry();
    }

        /*
         * Code to run ONCE when the driver hits PLAY
         */
        @Override
        public void start () {
        runtime.reset();
    }

        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */
        @Override
        public void loop () {
        robot.SwerveDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        robot.UpdateTelemetry();
    }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop () {
        robot.KillMotors();
    }

}
