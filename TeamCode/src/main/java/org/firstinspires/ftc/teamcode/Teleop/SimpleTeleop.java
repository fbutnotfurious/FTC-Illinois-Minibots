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
// - - - - - - - - - - Imports - - - - - - - - - - - - -

@Config
@TeleOp(group = "Teleop")
public class SimpleTeleop extends LinearOpMode {

    //- - - - - - - - - - - - - - Initialization of Variables - - - - - - - - - - - -
    // - - - Timer for the teleop period - - - //
    private ElapsedTime teleopTimer = new ElapsedTime();
    private static final float TELEOP_TIME_OUT = 140; // Match time limit
    // - - - Timer for the teleop period - - - //

    // - - - Constants + Variables - - - //
    private static final double RAISE_SPEED = 1.0; // Full speed for raising
    private DcMotorEx ArmMotor;
    private DcMotorEx TwoStageMotor;
    private ArmControl armControl;
    private Gripper gripper;
    private double speedFactor = 0.5;
    private boolean holdingPosition = false; // Tracking if arm is in hold mode
    // - - - Constants + Variables - - - //
    //- - - - - - - - - - - - - - Initialization - - - - - - - - - - - -

    FtcDashboard dashboard;


    @Override
    public void runOpMode() throws InterruptedException {

        // - - - - - - - - - - Initialize components - - - - - - - - - -

        // - - - Setting up Mecanum Drive - - - //
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // - - - Setting up Mecanum Drive - - - //

        // - - - Setting up arm, two-stage motors, and gripper - - - //
        ArmMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        TwoStageMotor = hardwareMap.get(DcMotorEx.class, "TwoStageMotor");
        armControl = new ArmControl(hardwareMap);
        gripper = new Gripper(this);
        // - - - Setting up arm and two-stage motors, and gripper - - - //


        // - - - Configuring motor modes and behaviors - - - //
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TwoStageMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TwoStageMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setDirection(DcMotor.Direction.REVERSE);
        // - - - Configuring motor modes and behaviors - - - //

        // - - - Set up dashboard telemetry - - - //
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // - - - Set up dashboard telemetry - - - //


        // - - - Initialize gripper to starting position - - - //
        gripper.init(hardwareMap);
        gripper.gripperStopped();
        gripper.setAnglerDown();
        // - - - Initialize gripper to starting position - - - //

        // - - - Waiting for start signal from driver station - - - //
        waitForStart();
        teleopTimer.reset();
        // - - - Waiting for start signal from driver station - - - //

        // - - - - - - - - - - Initialize components - - - - - - - - - -


        // - - - - - - - - - - Main Teleop Loop - - - - - - - - - -

        while (!isStopRequested()) {

            // - - - Mecanum drive control - - - //
            // Control the robot's movement with the gamepad 1's sticks
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.right_stick_y * speedFactor, // Forward/Backward Movement
                    -gamepad1.left_stick_x * speedFactor, // Strafing Left/right
                    -gamepad1.right_stick_x * speedFactor // Rotation
            ));
            // - - - Mecanum drive control - - - //


            // - - - Arm Control with Hold Position Feature - - - //
            double armPower = gamepad2.right_stick_y;
            if (Math.abs(armPower) > 0.1) {  // If driver moves the arm
                ArmMotor.setPower(armPower);
                holdingPosition = false; // Disable hold position mode
            } else if (!holdingPosition) {  // When driver releases control
                double currentArmPos = ArmMotor.getCurrentPosition() * (360.0 / 5 / 1425.1); // Adjust ticks to degrees
                armControl.setDesArmPosDeg(currentArmPos); // Set current position as target
                holdingPosition = true;
            }

            // If in hold position mode, maintain the arm at the set position
            if (holdingPosition) {
                armControl.update();  // PID control to hold position
            }
            // - - - Arm Control with Hold Position Feature - - - //


            // - - - Two-stage motor control - - - //
            // Controlling the two-stage motor using gamepad2's left and right triggers
            if (gamepad2.left_trigger > 0) {
                TwoStageMotor.setPower(gamepad2.left_trigger); // Extend
            } else if (gamepad2.right_trigger > 0) {
                TwoStageMotor.setPower(-gamepad2.right_trigger); // Retract
            } else {
                TwoStageMotor.setPower(0); // Stop if neither trigger is pressed
            }
            // - - - Two-stage motor control - - - //


            // - - - Gripper control - - - //
            // Control gripper andgle and position using gamepad2's buttons and left stick
            if (gamepad2.x) gripper.setAnglerUP(); // Set angler to up position
            if (gamepad2.y) gripper.setAnglerDown(); // Set angler to down position

            if (gamepad2.left_stick_x > 0.2) {
                gripper.gripperForward(gamepad2.left_stick_x); // Move gripper forward
            } else if (gamepad2.left_stick_x < -0.2) {
                gripper.gripperReverse(gamepad2.left_stick_x); // Move gripper backwards
            } else {
                gripper.gripperStopped(); // Stop gripper movement
            }
            // - - - Gripper control - - - //

            // - - - Telemetry Updates - - - //
            // Sending important data to telemetry to monitor
            telemetry.addData("Arm Position", ArmMotor.getCurrentPosition());
            telemetry.addData("Holding Position", holdingPosition);
            telemetry.addData("Elapsed Time", teleopTimer.time());
            telemetry.update();
            // - - - Telemetry Updates - - - //
        }
    }
}
