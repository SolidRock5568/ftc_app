package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardwareMap {

    /* public members */
    public DcMotor FrontLeftMotor = null;
    public DcMotor FrontRightMotor = null;
    public DcMotor BackLeftMotor = null;
    public DcMotor BackRightMotor = null;

    public Servo FrontLeftServo = null;
    public Servo FrontRightServo = null;
    public Servo BackLeftServo = null;
    public Servo BackRightServo = null;

    //public DcMotor InfeedMotor = null;

    //public DcMotor LiftMotorOne = null;
    //public DcMotor LiftMotorTwo = null;

    /* local members */
    HardwareMap hwMap = null;
    Telemetry telemetry = null;

    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        //Save a reference to Hardware map
        hwMap = ahwMap;

        // Get and initialize the motors
        FrontLeftMotor = initMotor("FrontLeftMotor", DcMotorSimple.Direction.FORWARD);
        FrontRightMotor = initMotor("FrontRightMotor", DcMotorSimple.Direction.FORWARD);
        BackLeftMotor = initMotor("BackLeftMotor", DcMotorSimple.Direction.FORWARD);
        BackRightMotor = initMotor("BackRightMotor", DcMotorSimple.Direction.FORWARD);

        //LiftMotorOne = initMotor("LiftMotorOne", DcMotorSimple.Direction.FORWARD);
        //LiftMotorTwo = initMotor("LiftMotorTwo", DcMotorSimple.Direction.FORWARD);

        //InfeedMotor = initMotor("InfeedMotor", DcMotorSimple.Direction.FORWARD);

        FrontLeftServo = initServo("FrontLeftServo", Direction.FORWARD);
        FrontRightServo = initServo("FrontRightServo", Direction.FORWARD);
        BackLeftServo = initServo("BackLeftServo", Direction.FORWARD);
        BackRightServo = initServo("BackRightServo", Direction.FORWARD);

    }

    public DcMotor initMotor(String hardwareName, DcMotorSimple.Direction direction) {
        DcMotor motor = hwMap.dcMotor.get(hardwareName);
            if (motor == null) {
                telemetry.addData("error", hardwareName + "not found");
                return null;
            }
            motor.setDirection(direction);
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return motor;

    }

    public Servo initServo(String hardwareName, Direction direction) {
        Servo servo = hwMap.servo.get(hardwareName);
        if (servo == null) {
            telemetry.addData("error", hardwareName + "not found");
            return null;
        }
        servo.setDirection(direction);
        return servo;
    }
}