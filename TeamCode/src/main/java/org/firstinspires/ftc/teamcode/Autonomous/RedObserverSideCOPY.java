package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Teleop.SimpleTeleop;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsytems.Gripper;

@Autonomous
//@Disabled
public class RedObserverSideCOPY extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        Gripper gripper= new Gripper(this);
        gripper.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //============================
        // ALL POSE SECTION
        //============================

        Pose2d startPos = new Pose2d(10,20,Math.toRadians(90));
        //Pose2d BlueCenterPark = new Pose2d(46,12,Math.toRadians(180));
        Pose2d BlueWallPark = new Pose2d(10,30,Math.toRadians(180));

        //============================
        // LEFT TRAJECTORY
        //============================
        TrajectorySequence StageBlueLeft = drive.trajectorySequenceBuilder(startPos)
                .waitSeconds(0.25)
                //allowing robot move to the begiining position
                .lineToLinearHeading(startPos)
                .waitSeconds(0.1)
                //move robot back 5
                .back(5)
                //
                //.UNSTABLE_addTemporalMarkerOffset(0.0,()->{gripper.gripperOpen();})
                .lineToLinearHeading(BlueWallPark)
                //.UNSTABLE_addTemporalMarkerOffset(0.0,()->{SimpleTeleop.gripperLeftOpen();})
                .forward(5)
                .lineToLinearHeading(startPos)
                //.addTemporalMarker(()->felipe.setAnglerLoad())
                .waitSeconds(0.25)
                //.UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setTurnerAutoLOAD();})
                // .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperWideOpen();})
                .waitSeconds(1)
                .back(15)
                //.UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperWideOpen();})
                .build();
        //move forward 20 in
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(20)
                .build();

        //move right while maintaining same heading for 20 in
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(20)
                .build();

        //move back for 20 in
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(20)
                .build();

        //move left while maintaining same heading for 20 in
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeLeft(20)
                .build();

        //move left while maintaining same heading for 20 in
        /*Trajectory traj5 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(3,0),Math.toRadians(0))
                .build();
        /*
        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d())
                //valid: .splineTo(new Vector2d(5,0),Math.toRadians(0))
                //valid: .splineTo(new Vector2d(0,5),Math.toRadians(0))
                //invalid: .splineTo(new Vector2d(0,0),Math.toRadians(45)) cannot trun with 0 distnace
                //invalid: .splineTo(new Vector2d(0,0),Math.toRadians(-45)) co negative heading
                .splineTo(new Vector2d(15,15),Math.toRadians(45))
              //  .splineTo(new Vector2d(0,30),Math.toRadians(0))
                .splineTo(new Vector2d(0,30),Math.toRadians(315))
                //  .splineTo(new Vector2d(5,5),Math.toRadians(0))
                .build();
        */

        waitForStart();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        //drive.followTrajectory(traj5);

         /*
        drive.followTrajectory(traj6);
        /*
        // temporal  marker checkout
        TrajectorySequence TestNow =drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(15,15,Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{gripper.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{gripper.setAnglerUP();})
                .lineToLinearHeading(new Pose2d(5,5,Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{gripper.gripperOpen();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{gripper.setAnglerDown();})
                .build();
        drive.followTrajectorySequence(TestNow);

         */
        drive.followTrajectorySequence(StageBlueLeft);

        telemetry.clearAll();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
}








/* notes for future refrences to servo/gripper
gripperClosed()
gripperOpen


*/