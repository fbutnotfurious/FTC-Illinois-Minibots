package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

// Customized classes and methods
import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.ArmControl;
import org.firstinspires.ftc.teamcode.subsytems.SliderControl;
import org.firstinspires.ftc.teamcode.subsytems.Gripper;

@Autonomous
public class RedBucketSide extends LinearOpMode {

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

        // Define the trajectory sequence
        TrajectorySequence StageRedBucket = drive.trajectorySequenceBuilder(startPos)
                // Step 1: Set angler down
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {armControl.setDesArmPosDeg(56);})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{gripper.setAnglerUP();})

                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> {sliderControl.setDesSliderLen(1);})
                .waitSeconds(3)
                .lineToLinearHeading(SpecimenDropoffPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{gripper.setAnglerDown();})


                .waitSeconds(1)
                .forward(6)
                // Step 5: Set Arm to the Deposit angle

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(45);})
                // Step 6: Slider motor to the Intake length
                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> {sliderControl.setDesSliderLen(4);})
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.gripperForward(0.3))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {gripper.gripperStopped();})
                .back(14)


                .strafeLeft(30)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{gripper.setAnglerUP();})
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(-15);})
                // turn 90 degree
                .turn(Math.toRadians(90))
                //Raise the arm
                .waitSeconds(0.1)
                .strafeRight(27)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setArmPower(0);})
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {sliderControl.setDesSliderLen(sliderControl.getSliderLen()+2);})
                .waitSeconds(1)
                //problem here
                //took offset of 0.2 and made it a waitSeconds function instead
                //changed factor 0.3->0.6 (look at gripper forward preset conditions, rather use bigger factor)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {gripper.gripperForward(0.3);})
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {gripper.gripperStopped();})

                //final build
                .build();

        // Wait for start signal
        waitForStart();


        // Execute the trajectory sequence
        drive.followTrajectorySequence(StageRedBucket);



    }
}