package org.firstinspires.ftc.teamcode.RoverRuckus;

public class TeleopConfig extends RobotHardwareMap
{
    public void InitServos(){
        SetServoPositions(0.5);
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

    private void SetServoPositions(double angle) {
        FrontLeftServo.setPosition(angle);
        FrontRightServo.setPosition(angle);
        BackLeftServo.setPosition(angle);
        BackRightServo.setPosition(angle);
    }

    public void SetLiftMotors(double liftPower) {
        LiftMotorOne.setPower(liftPower);
        LiftMotorTwo.setPower(liftPower);
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
}