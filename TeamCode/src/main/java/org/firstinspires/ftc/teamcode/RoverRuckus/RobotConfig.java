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


    /**
     * This function takes a value between 0 and 1 and adjusts it to be
     * between the min value and the max value
     * @param value Value to be unscaled
     * @param min Minimum value to be returned
     * @param max Maximum value to be returned
     * @return Returns the unscaled value between min and max
     */
    public double UnScaleValue(double value, double min, double max) {
        return (value * (max - min) + max);
    }
    
    /**
     * This function takes a value between min and max and adjusts it to be
     * between 0 and 1
     * @param value Value to be scaled
     * @param min Minimum value to be used
     * @param max Maximum value to be used
     * @return Returns a value between 0 and 1 scaled between min and max
     */
    public double ScaleValue(double value, double min, double max){
        return ((value - min)/(max - min));
    }

    public void SetServoPositions(double angle) {
        //We first have to scale our input value (joystick from -1 to 1) to a value between
        //0 and 1
        double scaledAngle = ScaleValue(angle, -1, 1);

        //Next we have to Unscale our 0 to 1 value to each servo's min to max value
        FrontLeftServo.setPosition(UnScaleValue(scaledAngle, FrontLeftMin, FrontLeftMax));
        FrontRightServo.setPosition(UnScaleValue(scaledAngle, FrontRightMin, FrontRightMax));
        BackLeftServo.setPosition(UnScaleValue(scaledAngle, BackLeftMin, BackLeftMax));
        BackRightServo.setPosition(UnScaleValue(scaledAngle, BackRightMin, BackRightMax));
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