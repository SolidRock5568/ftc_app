package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="DashboardTests", group="Concept")
//@Disabled
public class DashboardTests extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    RobotConfig robot = new RobotConfig();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    TelemetryPacket field = new TelemetryPacket();
    TelemetryPacket text = new TelemetryPacket();
    TelemetryPacket.Adapter packetAdapter = new TelemetryPacket.Adapter(dashboard);

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
    public void init_loop(){

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        dashboard.setTelemetryTransmissionInterval(20);
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        dashboard.sendTelemetryPacket(text);
        dashboard.sendTelemetryPacket(field);
        packet.put("PacketAdapter", packetAdapter);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){

    }

}

