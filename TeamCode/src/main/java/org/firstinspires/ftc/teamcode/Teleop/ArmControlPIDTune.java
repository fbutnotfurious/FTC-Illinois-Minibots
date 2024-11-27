package org.firstinspires.ftc.teamcode.Teleop;

// - - - - - - - - - - Imports - - - - - - - - - - - - -
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
import org.firstinspires.ftc.teamcode.subsytems.ArmControlPID;
// - - - - - - - - - - Imports - - - - - - - - - - - - -

@Config
@TeleOp(group = "Teleop")
public class ArmControlPIDTune extends LinearOpMode {

    //- - - - - - - - - - - - - - Initialization of Variables - - - - - - - - - - - -
    // - - - Timer for the teleop period - - - //
    private ElapsedTime teleopTimer = new ElapsedTime();
    private static final float TELEOP_TIME_OUT = 140; // Match time limit
    // - - - Timer for the teleop period - - - //

    // - - - Constants + Variables - - - //
    private DcMotorEx TwoStageMotor;
    private ArmControlPID armControl;
    private Gripper gripper;
    private double speedFactor = 0.45;
    private int LatchInd= 0;
    private boolean holdingPosition = false; // Tracking if arm is in hold mode
    // PID tune for ArmControl
    public static double tgtAngleDeg=20;
    private int[] newtgtticks;

    FtcDashboard dashboard;


    //- - - - - - - - - - - - - - Initialization - - - - - - - - - - - -

    @Override
    public void runOpMode() throws InterruptedException {

        // - - - - - - - - - - Initialize components - - - - - - - - - -

        // - - - Setting up Mecanum Drive - - - //
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // - - - Setting up Mecanum Drive - - - //

        // - - - Setting up arm, two-stage motors, and gripper - - - //
        TwoStageMotor = hardwareMap.get(DcMotorEx.class, "TwoStageMotor");
        armControl = new ArmControlPID();
        gripper = new Gripper(this);
        // - - - Setting up arm and two-stage motors, and gripper - - - //


        // - - - Configuring motor modes and behaviors - - - //
        TwoStageMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TwoStageMotor.setDirection(DcMotor.Direction.REVERSE);
        TwoStageMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // - - - Configuring motor modes and behaviors - - - //

        // - - - Set up dashboard telemetry - - - //

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // - - - Set up dashboard telemetry - - - //


        // - - - Initialize gripper to starting position - - - //
        gripper.init(hardwareMap);
        gripper.gripperStopped();
        gripper.setAnglerDown();

        // - - - Initialize armControl - - - //
        armControl.init(hardwareMap);

        // - - - Waiting for start signal from driver station - - - //
        waitForStart();
        teleopTimer.reset();
        // - - - Waiting for start signal from driver station - - - //

        // - - - - - - - - - - Initialize components - - - - - - - - - -


        // - - - - - - - - - - Main Teleop Loop - - - - - - - - - -

        while (!isStopRequested()) {

            newtgtticks =armControl.SetArmTargetAngle(tgtAngleDeg);

            // - - - Mecanum drive control - - - //
            // Control the robot's movement with the gamepad 1's sticks
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.right_stick_y * speedFactor, // Forward/Backward Movement
                    -gamepad1.left_stick_x * speedFactor, // Strafing Left/right
                    -gamepad1.right_stick_x * speedFactor // Rotation
            ));


            // - - - Arm Control with Latching in Position Feature - - - /



            // - - - Two-stage motor control - - - //
            // Controlling the two-stage motor using gamepad2's left and right triggers
            if (gamepad2.left_trigger > 0) {
                TwoStageMotor.setPower(-gamepad2.left_trigger); // Extend
            } else if (gamepad2.right_trigger > 0) {
                TwoStageMotor.setPower(gamepad2.right_trigger); // Retract

            } else {
                TwoStageMotor.setPower(0); // Stop if neither trigger is pressed
            }



            // - - - Gripper control - - - //
            // Control gripper angler and position using gamepad2's buttons and left stick
            if (gamepad2.x) gripper.setAnglerUP(); // Set angler to up position
            if (gamepad2.y) gripper.setAnglerDown(); // Set angler to down position

            if (gamepad2.left_stick_x > 0.1) {
                gripper.gripperForward(gamepad2.left_stick_x); // Move gripper forward
            } else if (gamepad2.left_stick_x < -0.1) {
                gripper.gripperReverse(gamepad2.left_stick_x); // Move gripper backwards
            } else {
                gripper.gripperStopped(); // Stop gripper movement
            }


            // - - - Telemetry Updates - - - //
            // Sending important data to telemetry to monitor
            telemetry.addData("Arm Position in Deg","%.3f", armControl.GetArmCurrentAngle());
            telemetry.addData("Arm New Tgt Tick Cur1", newtgtticks[0]);
            telemetry.addData("Arm New Tgt Tick Cur 2", newtgtticks[1]);
            telemetry.addData("Holding Position", LatchInd);
            telemetry.addData("Elapsed Time", "%.3f", teleopTimer.time());
            telemetry.addData("TwoStage Position", TwoStageMotor.getCurrentPosition());
            telemetry.update();

        }
    }
}