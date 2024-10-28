package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.Gripper;

@Config
@TeleOp(group = "Teleop")

public class SimpleTeleop extends LinearOpMode {
    private ElapsedTime teleopTimer = new ElapsedTime();
    private final float TELEOP_TIME_OUT = 140; // WARNING: LOWER FOR OUTREACH

    FtcDashboard dashboard;
    Gripper gripper = new Gripper(this);

    double speedFactor = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        // set up Mecanum Drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap); // this has to be here inside the runopmode. The others go above as class variables
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //gripper.init(hardwareMap);


        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ////////////////////////////////////////////////////////////////////////////////////////////
        // WAIT FOR MATCH TO START
        ///////////////////////////////////////////////////////////////////////////////////////////

       // gripper.gripperClosed();
       // gripper.setAnglerDown();

        waitForStart();
        teleopTimer.reset();

        while (!isStopRequested() && teleopTimer.time() < TELEOP_TIME_OUT) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.right_stick_y * speedFactor,
                            -gamepad1.left_stick_x * speedFactor,
                            -gamepad1.right_stick_x * speedFactor
                    )
            );

            /*if(gamepad1.right_trigger > 0.25){
                gripper.gripperOpen();
            }

            if(gamepad1.left_trigger > 0.25){
                gripper.gripperClosed();
            }

            if(gamepad1.right_bumper){
                gripper.setAnglerUP();
            }

            if(gamepad1.left_bumper){
                gripper.setAnglerDown();
            }*/
        }
    }
}