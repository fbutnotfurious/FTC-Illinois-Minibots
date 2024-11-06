package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Teleop.ArmControl;
@Config
@TeleOp(name = "Arm Control Test")
public class ArmControlTestOpMode extends LinearOpMode {

    private ArmControl armControl;

    @Override
    public void runOpMode() {
        armControl = new ArmControl(hardwareMap); // Initialize with hardware map

        waitForStart();

        while (opModeIsActive()) {
            armControl.setDesArmPosDeg(45.0); // Set target position in degrees
            armControl.update(); // Update PIDF control to reach target position

            telemetry.addData("Desired Position", 45.0);
            telemetry.addData("Actual Position", armControl.getActArmPosDeg()); // Add a getter if needed
            telemetry.update();
        }
    }
}