package org.firstinspires.ftc.teamcode.RoverRuckus;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.lang.Math;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


public class RobotConfig extends RobotHardwareMap
{
    private static final String VUFORIA_KEY = "AUqb0l//////AAAAGevdjhGJj0rPhL7HPPOgXiMjObqiWrCOBJv2OvmyVIE1WTBpDt2ccEX7yWqDCZNRiMvT3ZeM/aA/Qx5Jpd1+8EraKY+8FD/uFVHJmMMwfkcYJkuIz3NzoVTdSc7c/3lwVmt7APWF/KAhoD6OPaoEjh1+gE17QlLkUoQNhlEEbbG3o2gjkyQf2xC+ZbXVehs0DF+ilZzliIa0NHNBSXutZaVmOzdbzJQNioxv9U+kf6P61pEy3aHvBPsqmRatPjzOeEN+/7NVyFJiDk2iakWxIrlTF0jUWl9zFBJcbXM+AwAaC57xY+txkzO8WFDR/ZQygDUajJKZQbfk+AbUj4yVMDCrX2bmrmZDAOFvrFtVFlqZ";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;

    //These are the value to use for each module to get that module to the zero degree
    public double FrontLeftMin = 0.05;
    public double FrontRightMin = 0.02;
    public double BackLeftMin = 0.01;
    public double BackRightMin = 0.03;

    public double FrontLeftMax = 0.69;
    public double FrontRightMax = 0.76;
    public double BackLeftMax = 0.70;
    public double BackRightMax = 0.79;

    private VuforiaTrackables targetsRoverRuckus = null;
    private List<VuforiaTrackable> allTrackables = null;

    public void VuforiaInit(){
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = this.hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");

        // Gets and sets the name of the "Blue Rover" trackable
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");

        // Gets and sets the name of the "Red Footprint" trackable
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");

        // Gets and sets the name of the "Front Craters" trackable
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");

        // Gets and sets the name of the "Back Space" trackable
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<>();
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
    }

    public String VuforiaRun(){
        int counter = 0;
        String ReturnValue = "";
        VuforiaTrackableDefaultListener trackableDefaultListener;
        targetsRoverRuckus.activate();
        if (counter <= 100) {
            // check all the trackable target to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                trackableDefaultListener = (VuforiaTrackableDefaultListener)trackable.getListener();
                if (trackableDefaultListener.isVisible()) {
                    targetVisible = true;
                    ReturnValue = trackable.getName();
                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = trackableDefaultListener.getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }

                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();

                float x = translation.get(0) / mmPerInch;
                float y = translation.get(1) / mmPerInch;
                float z = translation.get(2) / mmPerInch;

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            } else {
                //return name
            }
            counter++;
        }
        return ReturnValue;
    }

    public void dump() {
        //raise lift
        //flip over arm
        //check colors
        //find positon with Vuforia
        VuforiaRun();
        //flip out blocks/cubes
    }

    public void InitServos(){
        FrontLeftServo.setPosition(FrontLeftMin);
        FrontRightServo.setPosition(FrontRightMin);
        BackLeftServo.setPosition(BackLeftMin);
        BackRightServo.setPosition(BackRightMin);
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
        SetLeftDriveMotors(-Power + Steer);
        SetRightDriveMotors(-Power - Steer);
        SetSwerveServoPositions(Strafe);
    }

    public void FancySwerve(double Power, double Strafe, double Steer){
        double pi = 3.141592653589793;

        double ForwardPower = -Power;
        double StrafeRightPower = Strafe;
        double RotateClockWisePower = Steer;

        double Length = 10.8947877002801;
        double Width = 13.21;
        double DiagonalDistance = Math.sqrt(Math.pow(Length, 2)+ Math.pow(Width, 2));

        double A = StrafeRightPower - RotateClockWisePower*(Length/DiagonalDistance);
        double B = StrafeRightPower + RotateClockWisePower*(Length/DiagonalDistance);
        double C = ForwardPower - RotateClockWisePower*(Width/DiagonalDistance);
        double D = ForwardPower + RotateClockWisePower*(Width/DiagonalDistance);

        double FrontRightSpeed = Math.sqrt(Math.pow(B, 2)+Math.pow(C, 2));
        double FrontLeftSpeed = Math.sqrt(Math.pow(B, 2)+Math.pow(D, 2));
        double BackLeftSpeed = Math.sqrt(Math.pow(A, 2)+Math.pow(D, 2));
        double BackRightSpeed = Math.sqrt(Math.pow(A, 2)+Math.pow(C, 2));

        double FrontRightAngle = Math.atan2(B,C)*180/pi;
        double FrontLeftAngle = Math.atan2(B,D)*180/pi;
        double BackLeftAngle = Math.atan2(A,D)*180/pi;
        double BackRightAngle = Math.atan2(A,C)*180/pi;

        double FrontRightSpeedAdjusted = 0;
        double FrontLeftSpeedAdjusted = 0;
        double BackLeftSpeedAdjusted = 0;
        double BackRightSpeedAdjusted = 0;

        double FrontRightAngleAdjusted = 0;
        double FrontLeftAngleAdjusted = 0;
        double BackLeftAngleAdjusted = 0;
        double BackRightAngleAdjusted = 0;

        if(FrontRightAngle > 90 || FrontRightAngle <90) {FrontRightSpeedAdjusted = -1 * FrontRightSpeed;}
        if(FrontLeftAngle > 90 || FrontLeftAngle <90) {FrontLeftSpeedAdjusted = -1 * FrontLeftSpeed;}
        if(BackLeftAngle > 90 || BackLeftAngle <90) {BackLeftSpeedAdjusted = -1 * BackLeftSpeed;}
        if(BackRightAngle > 90 || BackRightAngle <90) {BackRightSpeedAdjusted = -1 * BackRightSpeed;}

        if(FrontRightAngle > 90) {FrontRightAngleAdjusted -= 180;} else if (FrontRightAngle < 90) {FrontRightAngleAdjusted += 180;}
        if(FrontLeftAngle > 90) {FrontLeftAngleAdjusted -= 180;} else if (FrontLeftAngle < 90) {FrontLeftAngleAdjusted += 180;}
        if(BackLeftAngle > 90) {BackLeftAngleAdjusted -= 180;} else if (BackLeftAngle < 90) {BackLeftAngleAdjusted += 180;}
        if(BackRightAngle > 90) {BackRightAngleAdjusted -= 180;} else if (BackRightAngle < 90) {BackRightAngleAdjusted += 180;}

        double max = FrontRightSpeed;
        if(FrontLeftSpeed>max){max = FrontLeftSpeed;}
        if(BackLeftSpeed>max){max = BackLeftSpeed;}
        if(BackRightSpeed>max){max = BackRightSpeed;}

        if(max>1){
            FrontRightSpeed/=max;
            FrontLeftSpeed/=max;
            BackLeftSpeed/=max;
            BackRightSpeed/=max;
        }

        //Set motor power
        FrontLeftMotor.setPower(FrontLeftSpeedAdjusted);
        FrontRightMotor.setPower(FrontRightSpeedAdjusted);
        BackLeftMotor.setPower(BackLeftSpeedAdjusted);
        BackRightMotor.setPower(BackRightSpeedAdjusted);

        //Set Servo positions
        FrontLeftServo.setPosition(UnScaleValue(ScaleValue(FrontRightAngleAdjusted, -90, 90), FrontLeftMin, FrontLeftMax));
        FrontRightServo.setPosition(UnScaleValue(ScaleValue(FrontLeftAngleAdjusted, -90, 90), FrontRightMin, FrontRightMax));
        BackLeftServo.setPosition(UnScaleValue(ScaleValue(BackLeftAngleAdjusted, -90, 90), BackLeftMin, BackLeftMax));
        BackRightServo.setPosition(UnScaleValue(ScaleValue(BackRightAngleAdjusted, -90, 90), BackRightMin, BackRightMax));
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
        return (value * (max - min) + min);
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

    public void SetSwerveServoPositions(double angle) {
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