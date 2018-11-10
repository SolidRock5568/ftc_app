package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name="TeleopWebcam", group="RoverRuckus")
//@Disabled
public class TeleopWebcam extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    TelemetryPacket.Adapter dashboardTelemetry = new TelemetryPacket.Adapter(dashboard);

    double LeftJoystickY = 0;
    double LeftJoystickX = 0;
    double RightJoystickX = 0;

    double LeftEncoder = 0;
    double RightEncoder = 0;

    WebcamName SideCamera;
    WebcamName FrontCamera;

    SamplingOrderDetector SamplingDetector;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //dashboard.setTelemetryTransmissionInterval(100);
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if(!gamepad1.a)
        {
            telemetry.addData("Order", Sampling());
        }
        dashboardTelemetry.addData("Gamepad 1 A", gamepad1.a);
//        packet.put("PacketAdapter", dashboardTelemetry);
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public String Sampling() {
        String order = null;

        // Setup detector
        SamplingDetector = new SamplingOrderDetector(); // Create the detector
        SamplingDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1, false); // Initialize detector with app context and camera
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

}
