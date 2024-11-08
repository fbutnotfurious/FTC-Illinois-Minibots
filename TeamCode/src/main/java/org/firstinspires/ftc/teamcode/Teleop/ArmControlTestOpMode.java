package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Teleop.ArmControl;
@Config
@TeleOp(name = "Arm Control Test")
public class ArmControlTestOpMode extends LinearOpMode {

    /*// PID and Feedforward Coefficients
    public static double kP = 0.1; // Proportional Gain
    public static double kI = 0.0; // Integral Gain
    public static double kD = 0.0; // Derivative Gain
    public static double kS = 0.0; // Static friction compensation
    public static double kG = 0.0; // Gravity compensation*/
    public static double desDeg=20;

    private ArmControl armControl;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        // - - - Set up dashboard telemetry - - - //
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // - - - Set up dashboard telemetry - - - //

        armControl = new ArmControl(hardwareMap); // Initialize with hardware map

        waitForStart();

        while (opModeIsActive()) {
            armControl.setDesArmPosDeg(desDeg); // Set target position in degrees
           // armControl.update(kP,kI,kD,kS,kG); // Update PIDF control to reach target position
            armControl.update();
            telemetry.addData("Desired Position", desDeg);
            telemetry.addData("Actual Position", armControl.getActArmPosDeg()); // Add a getter if needed
            telemetry.update();
        }
    }
}