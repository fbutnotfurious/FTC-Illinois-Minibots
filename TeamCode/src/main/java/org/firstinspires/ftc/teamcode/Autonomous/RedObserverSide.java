package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Teleop.ArmControl;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsytems.Gripper;

@Autonomous
public class RedObserverSide extends LinearOpMode {
    private DcMotorEx ArmMotor;
    private DcMotorEx TwoStageMotor;
    private ArmControl armControl;
    private Gripper gripper;
    private double speedFactor = 0.65;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Mecanum Drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);

        // Initialize telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Define starting position
        Pose2d startPos = new Pose2d(10, 20, Math.toRadians(90));
        drive.setPoseEstimate(startPos);

        // Define the trajectory sequence
        TrajectorySequence StageRedObserver = drive.trajectorySequenceBuilder(startPos)
                // Step 1: Set angler down
                //.UNSTABLE_addTemporalMarkerOffset(0.0,()->{gripper.setAnglerDown();})
                .forward(3)
                // Step 2: Turn left by 90 degrees
                //.turn(Math.toRadians(90))

                // Step 3: Set angler up
                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> {gripper.setAnglerUP();})

                // Step 4: Move forward 60 inches
                //.forward(60)

                // Step 5: Open gripper
                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> gripper.gripperForward(1))

                // Step 6: Move back by 90 inches
                //.back(90)

                // Step 7: Turn right by 90 degrees
                //.turn(Math.toRadians(-90))
                .strafeRight(45)

                .build();

        // Wait for start signal
        waitForStart();

        // Execute the trajectory sequence
        drive.followTrajectorySequence(StageRedObserver);

        // Clear telemetry after completion
        telemetry.clearAll();
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}