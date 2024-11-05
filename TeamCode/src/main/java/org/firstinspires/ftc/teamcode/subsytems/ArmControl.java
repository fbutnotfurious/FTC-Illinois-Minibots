package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import static java.lang.Thread.sleep;;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import static java.lang.Thread.sleep;

public class ArmControl {
    public static double kS = 0; // TODO - tune static friction compensation
    public static double kG = 0;// TODO - tune gravity compensation
    public static double kP = 0;// TODO - tune closed loop Proportional Gain
    public static double kI = 0;// TODO - tune closed loop Integral Gain
    public static double kD = 0;// TODO - tune closed loop Derivative Gain

    /// constructor with opmode passed in
    public ArmControl(LinearOpMode opmode) {
        this.opmode = opmode;
    }
        private double actArmPosDeg = 0.0;
        private double desArmPosDeg = 10.0;



        private static double Ts = 0.01; // TODO - set this equal to the periodic rate (in seconds) that update() will get called at.

        private const double batteryVoltage = 12.0; //TODO - set this to the volts your battery normally is. Or, even better, periodically set it to the battery voltage you measure

        DcMotor ArmMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");// TODO - set up your motor here
        ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        private double errPrev = 0.0;
        private double errAccum = 0.0;

    }

    // TODO - call this method periodically elsewhere in code, to command the arm to a particular angle.
    public void setDesArmPosDeg(double desArmPosDeg){
        this.desArmPosDeg = desArmPosDeg;
    }

    // TODO - Call this periodically
    public void update(){
        this.actArmPosDeg = ArmMotor.;//TODO - read the encoder on the motor to get the current position in degrees, where 0 = level with the ground, positive is tilted up, and negative is tilted down.\

        double err = this.desArmPosDeg - this.actArmPosDeg;

        // Implement an arm-with-friction feedforward control
        double staticFrictionCompensation = Math.sign(err) * this.kS;
        double gravityCompensation = Math.cos(this.actArmPosDeg)* this.kG;
        double feedForwardVoltage = staticFrictionCompensation + gravityCompensation;

        // Implement a generic PID controller for feedback control
        this.errAccum += err * this.Ts;
        double pTerm = err * this.kP;
        double iTerm = this.errAccum * this.kI;
        double dTerm = (err - this.errPrev)/this.Ts * this.kD;
        this.errPrev = err;
        double feedBackVoltage = pTerm + iTerm + dTerm;

        // Calculate the "power" command for the motor as a fraction of available battery voltage.
        double motorCmd = (feedBackVoltage + feedForwardVoltage) / this.batteryVoltage;
        this.armMotor.setpower(motorCmd)

    }
}
