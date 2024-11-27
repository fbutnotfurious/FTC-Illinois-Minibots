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
    private double speedFactor = 0.45;
    private int LatchInd= 0;
    private boolean holdingPosition = false; // Tracking if arm is in hold mode
    FtcDashboard dashboard;
    // - - - Constants + Variables - - - //
    //- - - - - - - - - - - - - - Initialization - - - - - - - - - - - -

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
        TwoStageMotor.setDirection(DcMotor.Direction.REVERSE);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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


            // - - - Arm Control with Latching in Position Feature - - - /
            if(Math.abs(gamepad2.right_stick_y) > 0.1){
                ArmMotor.setPower(-1.0*gamepad2.right_stick_y*0.65);
            }
            else {
                ArmMotor.setPower(0);
            }

            // A button for latching in current position
            if (gamepad2.a) {
                // Set latching indication to 1 to maintain the arm at the current position
                LatchInd=1;
            }
            if(LatchInd ==1){
                // Holding the Arm in position when LatchInd is 1
                armControl.latch();  // PID control to hold position
            }
            // Release latching when gampad2 B button is pushed
            if (gamepad2.b) {
                LatchInd=0;
                //ArmMotor.setPower(0);
            }


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
            telemetry.addData("Arm Position", ArmMotor.getCurrentPosition());
            telemetry.addData("Arm Motor Power", "%.2f",ArmMotor.getPower());
            telemetry.addData("Holding Position", LatchInd);
            telemetry.addData("Elapsed Time", "%.3f", teleopTimer.time());
            telemetry.addData("TwoStage Position", TwoStageMotor.getCurrentPosition());
            telemetry.update();

        }
    }
}