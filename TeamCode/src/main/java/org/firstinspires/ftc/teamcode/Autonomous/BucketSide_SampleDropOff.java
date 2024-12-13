package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.ArmControl;
import org.firstinspires.ftc.teamcode.subsytems.Gripper;
import org.firstinspires.ftc.teamcode.subsytems.SliderControl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class BucketSide_SampleDropOff extends LinearOpMode {

    private ElapsedTime AutoTimer = new ElapsedTime();
    private ArmControl armControl;
    private Gripper gripper;
    private SliderControl sliderControl;
    private double speedFactor = 0.65;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Mecanum Drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // - - - Setting up Arm motors - - - //
        armControl = new ArmControl(this);
        armControl.init(hardwareMap);

        // - - - Setting up Slider motors - - - //
        sliderControl = new SliderControl(this);
        sliderControl.init(hardwareMap);

        // - - - Initialize gripper to starting position - - - //
        gripper = new Gripper(this);
        gripper.init(hardwareMap);
        gripper.gripperStopped();
        gripper.setAnglerDown();


        // Initialize telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Define starting position
        Pose2d startPos = new Pose2d(8, 87, Math.toRadians(0));
        drive.setPoseEstimate(startPos);

        Pose2d SpecimenDropoffPos = new Pose2d(33, 74, Math.toRadians(0));
        Pose2d SampleDropoffPos = new Pose2d(16, 126, Math.toRadians(135));

        // Define the trajectory sequence
        TrajectorySequence StageRedBucket = drive.trajectorySequenceBuilder(startPos)

                // Step 1: Set angler down
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(60);})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{gripper.setAnglerUP();})

                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> {sliderControl.setDesSliderLen(1);})
                .waitSeconds(0.5)
                .lineToLinearHeading(SpecimenDropoffPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{gripper.setAnglerDown();})


                //.waitSeconds(1)
                .forward(7)
                // Step 5: Set Arm to the Deposit angle

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(45);})
                // Step 6: Slider motor to the Intake length
                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> {sliderControl.setDesSliderLen(4);})
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.gripperForward(0.3))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {gripper.gripperStopped();})
                .back(5)
                //.waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.gripperReverse(-0.3))
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {gripper.gripperStopped();})


                .back(10)

                .strafeLeft(30)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{gripper.setAnglerUP();})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(-15);})
                // turn 90 degree
                .turn(Math.toRadians(90))
                //Raise the arm
                .waitSeconds(0.1)
                .strafeRight(27)

                // Pose2d for precise positioning //measure it
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setArmPower(0);}).UNSTABLE_addTemporalMarkerOffset(0.2, () -> {sliderControl.setDesSliderLen(sliderControl.getSliderLen()+5);})
                .waitSeconds(1)

                // Try to roll in the sample
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {gripper.gripperForward(0.3);})
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {gripper.gripperStopped();})
                // The following wait time is important to let the upper command sequence complete.
                // Without this time window, the control will just stop here without any actual operation,
                // which was the case for the bucket side tuning with the gripper not rolling

                // ** Run to the Deposit position with the Arm in the Deposit angle and slider full extended
                .waitSeconds(1.2)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setArmDeposit();})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {sliderControl.setSliderDeposit();})
                .lineToLinearHeading(SampleDropoffPos)

                // Move forward 2 inch and roll out to deposit the sample
                .forward(2)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {gripper.gripperReverse(-0.3);})
                //.UNSTABLE_addTemporalMarkerOffset(0.9, () -> {gripper.gripperStopped();})


                .waitSeconds(1)


                //final build
                .build();

        // Wait for start signal
        waitForStart();


        // Execute the trajectory sequence
        drive.followTrajectorySequence(StageRedBucket);



    }
}