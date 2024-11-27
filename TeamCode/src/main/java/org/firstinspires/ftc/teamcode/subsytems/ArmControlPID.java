package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;


@Config
public class ArmControlPID {

    ElapsedTime runtime = new ElapsedTime();

    // PID and Feedforward Coefficients
    public static double kP = 0.0; // Proportional Gain,
    public static double kI = 0.0; // Integral Gain
    public static double kD = 0.0; // Derivative Gain
    public  static double kS = 0.15; // Static friction compensation


    // Arm control variables
    private DcMotorEx armMotor;
    private double initialPosDeg=-22;
    private double RaiseSPEED=0.6;
    private double degreesPerTick = 360.0 / 5/1425.1;


    // Constructor to initialize the arm motor
    public void init(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients ArmPid_New = new PIDFCoefficients(kP, kI, kD, kS); // p was 10
        armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ArmPid_New);
    }

    public int[] SetArmTargetAngle(double TgtAngle){

        int newTargetTicks;

        int curTicks=0;

        int[] armInfo;
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Determine new target lift height in ticks based on the current position.
        // When the match starts the current position should be reset to zero.

        newTargetTicks = (int) ((TgtAngle - initialPosDeg) / degreesPerTick);
        // Set the target now that is has been calculated


        // Turn On RUN_TO_POSITION
        armMotor.setTargetPosition(newTargetTicks);
        armMotor.setPower(Math.abs(RaiseSPEED));
        // reset the timeout time and start motion.
        //runtime.reset();
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (armMotor.isBusy()){
            curTicks= armMotor.getCurrentPosition();

        };
        armInfo = new int[]{curTicks, newTargetTicks};
        return armInfo;

    }
    public double GetArmCurrentAngle( ){
        return armMotor.getCurrentPosition()*degreesPerTick+initialPosDeg;
    }

}