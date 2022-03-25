package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

/**
 * This opmode demonstrates how to create a teleop using just the SampleMecanumDrive class without
 * the need for an external robot class. This will allow you to do some cool things like
 * incorporating live trajectory following in your teleop. Check out TeleOpAgumentedDriving.java for
 * an example of such behavior.
 * <p>
 * This opmode is essentially just LocalizationTest.java with a few additions and comments.
 */
@Autonomous(group = "advanced")
public class BasicOcMode_5SouthWestCorner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        DcMotor spinner = null;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        spinner.setDirection(DcMotorSimple.Direction.FORWARD);
        // We want to turn off velocity control for auto
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(18)
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(4)
                .build();
        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        boolean stop = false;
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested() && !stop) {
            spinner.setPower(0.5);
            sleep(4000);
            spinner.setPower(0);
            drive.followTrajectory(trajectory);
            sleep(2000);
            drive.followTrajectory(trajectory2);
            stop = true;
        }
    }
}
