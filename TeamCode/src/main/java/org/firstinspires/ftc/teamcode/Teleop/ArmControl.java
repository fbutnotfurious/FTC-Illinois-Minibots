package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmControl {


    // Arm control variables
    private DcMotorEx ArmMotor;
    private double initialPosDeg=-20;
    private double DepositAngle=55;
    private double IntakeAngle= -10;

    private double RuntoPositionPower=0.3;


    private int desArmPosTick;
    private double degreesPerTick = 360.0 / 5/1425.1;

    // Constructor to initialize the arm motor
    public ArmControl(HardwareMap hardwareMap) {
        ArmMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public double getActArmPosDeg(){
        return ArmMotor.getCurrentPosition() * degreesPerTick + initialPosDeg;
    }

    public int getTgtArmPosTick(double TgtAngle){
        return (int)((TgtAngle-initialPosDeg)/degreesPerTick);
    }

    // Run to position for the desired Arm Angle
    public void setDesArmPosDeg(double tgtArmPosDeg) {
        desArmPosTick = (int)((tgtArmPosDeg-initialPosDeg)/degreesPerTick);

        ArmMotor.setTargetPosition(desArmPosTick);
        ArmMotor.setPower(RuntoPositionPower);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // Set Arm angle to the Deposit angle for deposit
    public void setArmDeposit() {
        desArmPosTick = (int)((DepositAngle-initialPosDeg)/degreesPerTick);
        ArmMotor.setTargetPosition(desArmPosTick);
        ArmMotor.setPower(RuntoPositionPower);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Set Arm angle to the intake angle for intake
    public void setArmIntake() {
        desArmPosTick = (int)((IntakeAngle-initialPosDeg)/degreesPerTick);
        ArmMotor.setTargetPosition(desArmPosTick);
        ArmMotor.setPower(RuntoPositionPower);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Wrapper method to set the ArmMotor power level to PwrLevel
    public void setArmPower(double PwrLevel){
        ArmMotor.setPower(PwrLevel);
    }

    // Wrapper method to get the ArmMotor power level
    public double getArmPower(){
        return ArmMotor.getPower();
    }

    // Wrapper method to set the ArmMotor run mode to using encoder
    public void ArmRunModReset(){
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setPower(0);
    }

}