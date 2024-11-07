package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
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
    public static double kG = 0.0; // Gravity compensation
*/
    private ArmControl armControl;

    @Override
    public void runOpMode() {
        armControl = new ArmControl(hardwareMap); // Initialize with hardware map

        waitForStart();

        while (opModeIsActive()) {
            armControl.setDesArmPosDeg(80.0); // Set target position in degrees
            //armControl.update(kP,kI,kD,kS,kG); // Update PIDF control to reach target position
            armControl.update();
            telemetry.addData("Desired Position", 45.0);
            telemetry.addData("Actual Position", armControl.getActArmPosDeg()); // Add a getter if needed
            telemetry.update();
        }
    }
}