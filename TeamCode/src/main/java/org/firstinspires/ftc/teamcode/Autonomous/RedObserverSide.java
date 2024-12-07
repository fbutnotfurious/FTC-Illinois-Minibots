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
public class RedObserverSide extends LinearOpMode {

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
        Pose2d startPos = new Pose2d(3, 3, Math.toRadians(90));
        drive.setPoseEstimate(startPos);

        // Define the trajectory sequence
        TrajectorySequence StageRedObserver = drive.trajectorySequenceBuilder(startPos)
                // Step 1: Set angler down
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{gripper.setAnglerDown();})
                .forward(3)
                // Step 2: Turn left by 90 degrees
                .turn(Math.toRadians(90))

                // Step 3: Set angler up
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {gripper.setAnglerUP();})

                // Step 4: Move forward 60 inches
                .forward(2)

                // Step 5: Set Arm to the Deposit angle
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> armControl.setArmDeposit())
                .waitSeconds(3)
                // Step 6: Slider motor to the Intake length
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> sliderControl.setSliderIntake())
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> gripper.gripperReverse(-0.3))
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> sliderControl.setDesSliderLen(1))
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> gripper.gripperStopped())
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> gripper.gripperForward(0.3))
                .waitSeconds(1)
                // Step 7: Move back by 90 inches
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> armControl.setArmIntake())
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> gripper.gripperStopped())
                .waitSeconds(2 )
                .back(2)
                .waitSeconds(3)


                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> armControl.setDesArmPosDeg(-10))

                .waitSeconds(3)

                // Step 7: Turn right by 90 degrees
                //.turn(Math.toRadians(-90))

                .strafeRight(2)

                .build();

        // Wait for start signal
        waitForStart();

        // Execute the trajectory sequence
        drive.followTrajectorySequence(StageRedObserver);



    }
}