package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.Gripper;

@Config
@TeleOp(group = "Teleop")

public class SimpleTeleop extends LinearOpMode {
    private ElapsedTime teleopTimer = new ElapsedTime();
    private final float TELEOP_TIME_OUT = 140; // WARNING: LOWER FOR OUTREACH
    private static final double           RaiseSPEED                  = 1.0; // full spee
    private DcMotorEx ArmMotor;
    private DcMotorEx TwoStageMotor;
    FtcDashboard dashboard;
    ElapsedTime runtime = new ElapsedTime();
    Gripper gripper = new Gripper(this);

    double speedFactor = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        // set up Mecanum Drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap); // this has to be here inside the runopmode. The others go above as class variables
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        TwoStageMotor = hardwareMap.get(DcMotorEx.class, "TwoStageMotor");


        gripper.init(hardwareMap);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TwoStageMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ////////////////////////////////////////////////////////////////////////////////////////////
        // WAIT FOR MATCH TO START
        ///////////////////////////////////////////////////////////////////////////////////////////

       gripper.gripperStopped();
       gripper.setAnglerDown();

        waitForStart();
        teleopTimer.reset();

        while (!isStopRequested()) { //&& teleopTimer.time() < TELEOP_TIME_OUT
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.right_stick_y * speedFactor,
                            -gamepad1.left_stick_x * speedFactor,
                            -gamepad1.right_stick_x * speedFactor
                    )
            );

            ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            TwoStageMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            /*
            int newTargetHeight;
            newTargetHeight = (int)(2600);
            // Set the target now that is has been calculated
            ArmMotor.setTargetPosition(newTargetHeight);
            // Turn On RUN_TO_POSITION
            ArmMotor.setPower(Math.abs(RaiseSPEED));
            // reset the timeout time and start motion.
            ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Set the target now that is has been calculated
            //ArmMotor.setTargetPosition(70);*/


            double currentPos;
            if (gamepad2.left_bumper ) {
                ArmMotor.setPower(0.4);
                currentPos = ArmMotor.getCurrentPosition();
                telemetry.addData("ArmPosition",currentPos);
                telemetry.update();
            }
            else if (gamepad2.right_bumper) {
                ArmMotor.setPower(-0.4);
                currentPos = ArmMotor.getCurrentPosition();
                telemetry.addData("ArmPosition",currentPos);
                telemetry.update();
            } else {
                ArmMotor.setPower(0.0);
            }

            if (gamepad2.left_trigger >0) {
                TwoStageMotor.setPower(gamepad2.left_trigger);
                currentPos = TwoStageMotor.getCurrentPosition();
                telemetry.addData("TwoStagePosition",currentPos);
                telemetry.update();
            }
            else if (gamepad2.right_trigger > 0) {
                TwoStageMotor.setPower(-gamepad2.right_trigger);
                currentPos = TwoStageMotor.getCurrentPosition();
                telemetry.addData("TwoStagePosition",currentPos);
                telemetry.update();
            } else {
                TwoStageMotor.setPower(0.0);
            }

            if(gamepad2.x){
                gripper.setAnglerUP();
            }

            if(gamepad2.y){
                gripper.setAnglerDown();
            }

            if(gamepad2.left_stick_x > 0.2){
                gripper.gripperForward(gamepad2.left_stick_x);
            }
            else if(gamepad2.left_stick_x <-0.2){
                gripper.gripperReverse(gamepad2.left_stick_x);

            }
            else{
                gripper.gripperStopped();
            }
        }
    }
}
