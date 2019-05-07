package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

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

    public DcMotor InfeedMotor = null;
    public DcMotor FlipMotor = null;

    public DcMotor LiftMotorOne = null;
    public DcMotor LiftMotorTwo = null;

    public Servo LeftDumpServo = null;
    public Servo RightDumpServo = null;


    /* local members */
    HardwareMap hwMap = null;
    Telemetry telemetry = null;

    AnalogInput ArmRotation;

    public void init(HardwareMap ahwMap, Telemetry atelemetry) {
        //Save a reference to Hardware map
        hwMap = ahwMap;
        telemetry = atelemetry;

        ArmRotation = hwMap.analogInput.get("Arm Potentiometer");

        // Get and initialize the motors
        FrontLeftMotor = initMotor("FrontLeftMotor", DcMotorSimple.Direction.REVERSE);
        FrontRightMotor = initMotor("FrontRightMotor", DcMotorSimple.Direction.FORWARD);
        BackLeftMotor = initMotor("BackLeftMotor", DcMotorSimple.Direction.REVERSE);
        BackRightMotor = initMotor("BackRightMotor", DcMotorSimple.Direction.FORWARD);

        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LiftMotorOne = initMotor("LiftMotorOne", DcMotorSimple.Direction.REVERSE);
        LiftMotorTwo = initMotor("LiftMotorTwo", DcMotorSimple.Direction.FORWARD);

        LiftMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        LiftMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        LiftMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        LiftMotorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        LiftMotorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

          InfeedMotor = initMotor("InfeedMotor", DcMotorSimple.Direction.REVERSE);

          FlipMotor = initMotor("FlipMotor", DcMotorSimple.Direction.FORWARD);

          FlipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        FlipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeftServo = initServo("FrontLeftServo", Direction.FORWARD);
        FrontRightServo = initServo("FrontRightServo", Direction.FORWARD);
        BackLeftServo = initServo("BackLeftServo", Direction.FORWARD);
        BackRightServo = initServo("BackRightServo", Direction.FORWARD);

        LeftDumpServo = initServo("LeftDumpServo", Direction.FORWARD);
        RightDumpServo = initServo("RightDumpServo", Direction.FORWARD);

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