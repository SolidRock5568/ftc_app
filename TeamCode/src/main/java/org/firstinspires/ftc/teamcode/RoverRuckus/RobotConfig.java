package org.firstinspires.ftc.teamcode.RoverRuckus;

import static com.qualcomm.robotcore.util.Range.clip;


public class RobotConfig extends RobotHardwareMap
{
    double FrontLeftMin = 0;
    double FrontRightMin = 0.02;
    double BackLeftMin = 0.01;
    double BackRightMin = 0.03;

    double FrontLeftMax = 1;
    double FrontRightMax = 0.76;
    double BackLeftMax = 0.70;
    double BackRightMax = 0.79;

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

    public void SetServoPositions(double angle) {
        FrontLeftServo.setPosition(angle * (FrontLeftMax - FrontLeftMin));
        FrontRightServo.setPosition(angle * (FrontRightMax - FrontRightMin));
        BackLeftServo.setPosition(angle * (BackLeftMax - BackLeftMin));
        BackRightServo.setPosition(angle * (BackRightMax - BackRightMin));
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
        FrontLeftMotor.setPower(leftPower);
        BackLeftMotor.setPower(leftPower);
    }

    public void SetRightDriveMotors(double rightPower) {
        FrontRightMotor.setPower(rightPower);
        BackRightMotor.setPower(rightPower);
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