package org.firstinspires.ftc.teamcode.Teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.Gripper;
@Config
@TeleOp(group = "Teleop")

public class ArmPidTune extends OpMode {
    @Override
    public void init() {
        public ArmControl(){
            private double actArmPosDeg = 0.0;
            private double desArmPosDeg = 10.0;

            private const double kS = // TODO - tune static friction compensation
            private const double kG = // TODO - tune gravity compensation
            private const double kP = // TODO - tune closed loop Proportional Gain
            private const double kI = // TODO - tune closed loop Integral Gain
            private const double kD = // TODO - tune closed loop Derivative Gain

            private const double Ts = // TODO - set this equal to the periodic rate (in seconds) that update() will get called at.

            private const double batteryVoltage = 12.0; //TODO - set this to the volts your battery normally is. Or, even better, periodically set it to the battery voltage you measure

            DcMotor armMotor = // TODO - set up your motor here
                    armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            private double errPrev = 0.0;
            private double errAccum = 0.0;

        }

    }

    private void ArmControl() {
    }

    @Override
    public void loop() {

    }
}
