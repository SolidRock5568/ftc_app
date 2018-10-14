package org.firstinspires.ftc.teamcode.RoverRuckus;

import static com.qualcomm.robotcore.util.Range.clip;


public class RobotConfig extends RobotHardwareMap
{
    //These are the value to use for each module to get that module to the zero degree
    private double FrontLeftMin = 0;
    private double FrontRightMin = 0.02;
    private double BackLeftMin = 0.01;
    private double BackRightMin = 0.03;

    private double FrontLeftMax = 1;
    private double FrontRightMax = 0.76;
    private double BackLeftMax = 0.70;
    private double BackRightMax = 0.79;

    public void InitServos(){
        SetServoPositions(0);
    }

    public void KillMotors(){
        KillDriveMotors();
        KillLiftMotors();
    }

    private void KillDriveMotors() {
        SetDriveMotors(0);
    }

    private void KillLiftMotors() {
        SetLiftMotors(0);
    }

    public void UpdateTelemetry(){
//        telemetry.addData("TeamColor", Color);
//        telemetry.addData("Autonomous Selection", AutoSelect);
//        telemetry.addData("Field Position", Position);
//        telemetry.update();
    }

    public void SwerveDrive(double Power, double Strafe, double Steer){
        SetLeftDriveMotors(Power - Steer);
        SetRightDriveMotors(Power + Steer);
        SetServoPositions(Strafe);
    }

    //This function takes a value between 0 and 1 and adjusts it to be
    //between the min value and the max value
    public double ScaleValue(double value, double min, double max) {
        return (value * (max - min) + max);
    }

    public void SetServoPositions(double angle) {
        FrontLeftServo.setPosition(ScaleValue(angle, FrontLeftMax, FrontLeftMin));
        FrontRightServo.setPosition(ScaleValue(angle, FrontRightMax, FrontRightMin));
        BackLeftServo.setPosition(ScaleValue(angle, BackLeftMax, BackLeftMin));
        BackRightServo.setPosition(ScaleValue(angle, BackRightMax, BackRightMin));
    }

    public void SetLiftMotors(double liftPower) {
        //LiftMotorOne.setPower(liftPower);
        //LiftMotorTwo.setPower(liftPower);
    }

    public void SetDriveMotors(double drive) {
        SetLeftDriveMotors(drive);
        SetRightDriveMotors(drive);
    }

    public void SetLeftDriveMotors(double leftPower) {
        FrontLeftMotor.setPower(clip(leftPower, -1, 1));
        BackLeftMotor.setPower(clip(leftPower, -1, 1));
    }

    public void SetRightDriveMotors(double rightPower) {
        FrontRightMotor.setPower(clip(rightPower, -1, 1));
        BackRightMotor.setPower(clip(rightPower, -1, 1));
    }

    public void InfeedIn() {
        //InfeedMotor.setPower(-1);
    }

    public void InfeedOut() {
        //InfeedMotor.setPower(1);
    }

    public void InfeedStop() {
        //InfeedMotor.setPower(0);
    }


}