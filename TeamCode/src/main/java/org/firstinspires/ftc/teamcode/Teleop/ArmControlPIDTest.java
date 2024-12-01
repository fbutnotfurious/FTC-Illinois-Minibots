package org.firstinspires.ftc.teamcode.Teleop;

// - - - - - - - - - - Imports - - - - - - - - - - - - -
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
// - - - - - - - - - - Imports - - - - - - - - - - - - -

@Config
@TeleOp(group = "Teleop")
public class ArmControlPIDTest extends LinearOpMode {

    //- - - - - - - - - - - - - - Initialization of Variables - - - - - - - - - - - -
    // - - - Timer for the teleop period - - - //
    private ElapsedTime teleopTimer = new ElapsedTime();
    private static final float TELEOP_TIME_OUT = 140; // Match time limit
    // - - - Timer for the teleop period - - - //

    // - - - Constants + Variables - - - //
    // PID tune for ArmControl
    public static double tgtAngleDeg=10;
    public static double kP = 9; // Proportional Gain,
    public static double kI = 2; // Integral Gain
    public static double kD = 0.0; // Derivative Gain
    public  static double kS = 0.0; // Static friction compensation

    private DcMotorEx armMotor;

    private double initialPosDeg=-22;
    private double degreesPerTick = 360.0 / 5/1425.1;

    private int newTargetTicks;

    private PIDFCoefficients Default_Pid;

    FtcDashboard dashboard;

    //- - - - - - - - - - - - - - Initialization - - - - - - - - - - - -

    @Override
    public void runOpMode() throws InterruptedException {

        // - - - - - - - - - - Initialize components - - - - - - - - - -

        armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Default_Pid=armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER); // 10,3,0,0
        //PIDFCoefficients pidTurner_New = new PIDFCoefficients(kP, kI, kD, kS);
        //armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidTurner_New);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // - - - Set up dashboard telemetry - - - //

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // - - - Waiting for start signal from driver station - - - //
        waitForStart();
        teleopTimer.reset();
        // - - - Waiting for start signal from driver station - - - //

        // - - - - - - - - - - Initialize components - - - - - - - - - -


        // - - - - - - - - - - Main Teleop Loop - - - - - - - - - -

        while (!isStopRequested()) {


            newTargetTicks = (int) ((tgtAngleDeg - initialPosDeg) / degreesPerTick);

            // Turn On RUN_TO_POSITION
            //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //armMotor.setTargetPosition(newTargetTicks);
            armMotor.setPower(0.4);
            if ( teleopTimer.time() >20 && teleopTimer.time() <30){
                newTargetTicks=newTargetTicks+200;
            } else if(teleopTimer.time() >30){
                newTargetTicks=400;
            }

            armMotor.setTargetPosition(newTargetTicks);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            //runtime.reset();




            // - - - Telemetry Updates - - - //
            // Sending important data to telemetry to monitor
            telemetry.addData("Test Arm Position target in Deg","%.3f", tgtAngleDeg);
            telemetry.addData("Test Arm New Tgt Ticks", newTargetTicks);
            telemetry.addData("Test Arm New Tgt Ticks Get", armMotor.getTargetPosition());
            telemetry.addData("Test Arm current power", armMotor.getPower());
            telemetry.addData("Test rm current posi", armMotor.getCurrentPosition());
            telemetry.addData("Current Time", "%.3f",teleopTimer.time());
            telemetry.addData("Arm Tgt Pos tolerance in Ticks", armMotor.getTargetPositionTolerance());
            // Example to push all the data into one line
            telemetry.addLine("Default PID Info: ");
            telemetry.addData("KP", "%.3f",Default_Pid.p);
            telemetry.addData("KI", "%.3f",Default_Pid.i);
            telemetry.addData("KD", "%.3f",Default_Pid.d);
            telemetry.addData("KF", "%.3f",Default_Pid.f);
            telemetry.update();

        }
    }
}