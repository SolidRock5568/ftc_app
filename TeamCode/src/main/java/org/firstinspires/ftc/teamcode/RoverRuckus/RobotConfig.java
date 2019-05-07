package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;

import java.lang.Math;
import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.qualcomm.robotcore.util.Range.clip;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;



public class RobotConfig extends RobotHardwareMap
{
    private static final String VUFORIA_KEY = "AfQigb7/////AAABmddi2zOiSUlWnJ1HxTdfpihGzx/EuBykt9fn2zw7u8Dj8G7oQqEaEASdTJ0BkbPoLgUOoAFGO5ryTjJQadE8/Urk0+wPQQlX8QGUPRmK8mDmSGD+VE1W+UJQ+YXTQt2SV3DphWfPKnJIFSvvcpJ85FY0VkvXcSEYTayvMjiyTbXu1IRpeLid+vZktEDSbwBucsseLUpSW42dIohPuR+nW1r+IXjxfWRKkQdnyBK5Xw8/sMKwrOpJZAkxp3Z1tPnmvgUzAoR6BqjGqMO5OgaKu0APd901cz11vV1aTvNMnDfG4X+01u1k8IVB7ONZI1Im6g3tgTugavD3SZhP4vn9XLOU9jVhLaJAXWrWQXqXfIDQ\n";

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    // Vuforia variables
    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    // DogeCV detector
    GoldAlignDetector detector;
    SamplingOrderDetector SamplingDetector;

    //These are the value to use for each module to get that module to the zero degree
    public double FrontLeftMax = 0.75;
    public double FrontRightMax = 0.85;
    public double BackLeftMax = 0.97;
    public double BackRightMax = 0.86;

    public double FrontLeftMin = FrontLeftMax - 0.74;
    public double FrontRightMin = FrontRightMax - 0.8;
    public double BackLeftMin = BackLeftMax - 0.8;
    public double BackRightMin = BackRightMax - 0.8;

    WebcamName SideCamera;
    WebcamName FrontCamera;

    private VuforiaTrackables targetsRoverRuckus = null;

    public void VuforiaInit(){
        SideCamera = hwMap.get(WebcamName.class, "SideCamera");

        // Set up parameters for Vuforia
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Vuforia licence key
        parameters.fillCameraMonitorViewParent = true;

        // Set camera name for Vuforia config
        parameters.cameraName = SideCamera;

        // Create Dogeforia object
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        //Setup trackables
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
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

        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        // Activate the targets
        targetsRoverRuckus.activate();

        // Initialize the detector
        detector = new GoldAlignDetector();
        detector.init(hwMap.appContext,CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.downscale = 0.8;

        // Set the detector
        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
    }

    public String VuforiaRun(){
        String target = null;
        vuforia.start();
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                target = trackable.getName();
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
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
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
        return target;
    }

    public String Sampling() {
        String order = null;

        // Setup detector
        SamplingDetector = new SamplingOrderDetector(); // Create the detector
        SamplingDetector.init(hwMap.appContext, CameraViewDisplay.getInstance(), 1, false); // Initialize detector with app context and camera
        SamplingDetector.useDefaults(); // Set detector to use default settings

        SamplingDetector.downscale = 0.4; // How much to downscale the input frames

        // Optional tuning
        SamplingDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        SamplingDetector.maxAreaScorer.weight = 0.001;

        SamplingDetector.ratioScorer.weight = 15;
        SamplingDetector.ratioScorer.perfectRatio = 1.0;

        SamplingDetector.enable(); // Start detector
        for(int k = 0; k < 1000; k++)
        {
            order = SamplingDetector.getCurrentOrder().toString();
        }

        return order;
    }

    public void raiseLift(int target, int deadzone, double power) {
        while(LiftMotorTwo.getCurrentPosition() > target - deadzone/2 || LiftMotorTwo.getCurrentPosition() < target + deadzone/2)
        {
            LiftMotorTwo.setTargetPosition(target);
            LiftMotorTwo.setPower(power);
            LiftMotorOne.setPower(LiftMotorTwo.getPower());
        }
    }

    public void flipArmUp(double Potentiometer) {
        double Minimum = 1;
        double Maximum = 3.29;
        double MinimumPowerOffset = clip(1/(Maximum-Minimum), 0.05, 1);

        while(Potentiometer < Maximum)
        {
            SetArmMotor(-1*((Potentiometer-Maximum)*MinimumPowerOffset));
        }
        if(Potentiometer >= Maximum)
        {
            SetArmMotor(0);
        }
    }

    public void SetArmMotor(double power) {
        FlipMotor.setPower(power);
    }

    public void dump(double potentiometer) {
        raiseLift(1500, 10, .5);
        flipArmUp(potentiometer);
        //check colors
        //find positon with Vuforia
        //VuforiaRun();
        //flip out blocks/cubes
    }

    public void InitServos(){
        FrontLeftServo.setPosition(/*(FrontLeftMax - FrontLeftMin)/2*/ FrontLeftMin);
        FrontRightServo.setPosition(/*(FrontRightMax - FrontRightMin)/2*/ FrontRightMin);
        BackLeftServo.setPosition(/*(BackLeftMax - BackLeftMin)/2*/ BackLeftMin);
        BackRightServo.setPosition(/*(BackRightMax - BackRightMin)/2*/ BackRightMin);
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
        SetLeftDriveMotors(-Power - Steer);
        SetRightDriveMotors(-Power + Steer);
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
        double scaledAngle = ScaleValue(angle, -1 , 1);

        //Next we have to Unscale our 0 to 1 value to each servo's min to max value
        FrontLeftServo.setPosition(UnScaleValue(scaledAngle, FrontLeftMin, FrontLeftMax));
        FrontRightServo.setPosition(UnScaleValue(scaledAngle, FrontRightMin, FrontRightMax));
        BackLeftServo.setPosition(UnScaleValue(scaledAngle, BackLeftMin, BackLeftMax));
        BackRightServo.setPosition(UnScaleValue(scaledAngle, BackRightMin, BackRightMax));
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
        FrontLeftMotor.setPower(clip(leftPower, -1, 1));
        BackLeftMotor.setPower(clip(leftPower, -1, 1));
    }

    public void SetRightDriveMotors(double rightPower) {
        FrontRightMotor.setPower(clip(rightPower, -1, 1));
        BackRightMotor.setPower(clip(rightPower, -1, 1));
    }

    public void InfeedIn() {
        InfeedMotor.setPower(-1);
    }

    public void InfeedStop() {
        InfeedMotor.setPower(0);
    }

    public void InfeedOut() {
        InfeedMotor.setPower(1);
    }

}