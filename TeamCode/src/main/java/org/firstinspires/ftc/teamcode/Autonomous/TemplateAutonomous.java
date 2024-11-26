package org.firstinspires.ftc.teamcode.Autonomous;

// - - - - - - - - - - - - - - - - - - - - - - Imports - - - - - - - - - - - - - - - - - - - - - - //

import com.acmerobotics.roadrunner.geometry.Pose2d;
// Provides the ability to define a robot's position using x, y coordinates and heading (angle).

import com.acmerobotics.roadrunner.geometry.Vector2d;
// Used for handling 2D points in space, such as waypoints or targets.

import com.acmerobotics.roadrunner.trajectory.Trajectory;
// Used to define a specific movement path for the robot, like moving forward, strafing, or turning.

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// Marks this class as an autonomous OpMode for the FTC robot controller app.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// Allows the code to execute sequentially in a single thread, making it suitable for autonomous operations.

import com.qualcomm.robotcore.hardware.DcMotor;
// Provides access to motor hardware and basic methods like setPower() and setMode().

import com.qualcomm.robotcore.hardware.DcMotorEx;
// Extends DcMotor to include advanced functionality like setting velocity or accessing encoder data.

import org.firstinspires.ftc.teamcode.Teleop.ArmControl;
// A custom subsystem class (we wrote/will write) to manage arm control.

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
// drivetrain class, wrapping all drive operations such as moving, turning, and following trajectories.

import org.firstinspires.ftc.teamcode.subsytems.Gripper;
// Our ripper subsystem class, providing methods to open, close, or manipulate objects.

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import javax.tools.ForwardingFileObject;
// Extends trajectory functionality to handle multiple sequential paths and movements.

// - - - - - - - - - - - - - - - - - - - - - - Imports - - - - - - - - - - - - - - - - - - - - - - //

@Autonomous(name = "TemplateAutonomous")
// Template for Autonomous Mode
public class TemplateAutonomous extends LinearOpMode {

    // - - - - - - - - - - Define Variables Section - - - - - - - - - - //
    /*
    Declare any motors, subsystems, or variables needed for autonomous operation.
    Example:
        private DcMotorEx ArmMotor; // Arm motor with extended functionality
        private ArmControl armControl; // Manages arm control
        private Gripper gripper; // Manages gripper actions like open and close
    */
    // - - - - - - - - - - Define Variables Section Ends - - - - - - - - //

    @Override
    public void runOpMode() throws InterruptedException {

        // - - - - - - - - - - Initialization Section - - - - - - - - - - //

        // hardwareMap allows access to all hardware devices (motors, servos, sensors) configured in the FTC app.
        // Example of initializing other subsystems like a gripper or arm
        // Gripper gripper = new Gripper(this);
        // gripper.init(hardwareMap); // Initializes the gripper hardware

        // Initialize the drivetrain (required for movement)
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        Gripper gripper = new Gripper(this);
        gripper.init(hardwareMap);

        // Set the starting position of the robot
        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(90)); // (x, y, heading in radians)
        drive.setPoseEstimate(startPos); // Tells the drivetrain where the robot is starting

        // - - - - - - - - - - Initialization Section Ends - - - - - - - - - - //


        // - - - - - - - - - - Trajectory Building Section - - - - - - - - - - //

        // Define Trajectories
        Trajectory moveForward = drive.trajectoryBuilder(startPos)
                .forward(20)
                .build();
        Trajectory moveBackward = drive.trajectoryBuilder(moveForward.end())
                .back(20)
                .build();

        // Note: These trajectories can be combined into a sequence if needed.
        // - - - - - - - - - - Trajectory Building Section Ends - - - - - - - - - - //


        // - - - - - - - - - - Instructions Section - - - - - - - - - - //

        /*
         HINTS for Writing Custom Code:
            1. Use `drive.trajectoryBuilder()` to define how the robot moves.
                - `.forward(distance)` for forward movement.
                - `.back(distance)` for backward movement.
                - `.strafeLeft(distance)` to strafe left.
                - `.strafeRight(distance)` to strafe right.
            2. For more advanced movements, use `.lineToLinearHeading(Pose2d targetPose)`.
            3. If using a gripper:
                - Open the gripper: `gripper.open();`
                - Close the gripper: `gripper.close();`
            4. For turning:
                - Use `drive.turn(angleInRadians)` for precise turns.
                    Example: `drive.turn(Math.toRadians(90)); // Turns 90 degrees left`
            5. For an arm subsystem:
                - Move the arm to a specific angle: `armControl.setArmAngle(angleInDegrees);`
                    Example: `armControl.setArmAngle(45); // Moves arm to 45 degrees`
         */

        // - - - - - - - - - - Instructions Section Ends - - - - - - - - - - //


        // - - - - - - - - - - Waiting for Start Section - - - - - - - - - - //
        telemetry.addData("Status", "Waiting for Start..."); // Displays a message on the driver station
        telemetry.update(); // Updates telemetry data
        waitForStart(); // Waits for the user to press "Start" on the driver station

        if (isStopRequested()) return; // Exits if "Stop" is pressed
        // - - - - - - - - - - Waiting for Start Section Ends - - - - - - - - - - //


        // - - - - - - - - - - Executing Trajectories Section - - - - - - - - - - //
        drive.turn(Math.toRadians(90)); //Negative for left turn

        sleep(100);

        drive.followTrajectory(moveForward);
        sleep(100);
        gripper.gripperForward(1.0);
        sleep(100);
        gripper.gripperStopped();
        sleep(100);
        drive.turn(Math.toRadians(-180)); //Turn 180 degrees
        sleep(100);
        drive.followTrajectory(moveBackward);
        sleep(100);
        drive.turn(Math.toRadians(90));

        /*
        Example of combining trajectories into a sequence:
            TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .forward(20)
                .strafeRight(20)
                .build();
            drive.followTrajectorySequence(sequence); // Executes the entire sequence
         */
        // - - - - - - - - - - Executing Trajectories Section Ends - - - - - - - - - - //

        // - - - - - - - - - - Tips - - - - - - - - - - //
        /*
        Debugging and Optimization:
            1. Use telemetry to display robot position or variable values.
                Example: telemetry.addData("Robot Position", drive.getPoseEstimate());
            2. Break your program into smaller steps and test each step individually.
            3. Add delays (e.g., `sleep(1000);` for 1 second) to observe robot movements.
            4. Be mindful of field measurements and the robot's starting alignment.
         */
        telemetry.clearAll(); // Clears all telemetry messages after the program completes
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // - - - - - - - - - - Tips - - - - - - - - - - //
    }
}