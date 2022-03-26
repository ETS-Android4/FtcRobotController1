package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
@Autonomous(group = "org/firstinspires/ftc/teamcode/advanced")
public class BasicOcMode_6SouthWestCorner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetryUpdateLine(27);
        //Initialize SampleMecanumDrive
        Definitions robot = new Definitions();
        CRServo slideServo;
        DcMotor slide;
        DcMotor spinner;
        telemetryUpdateLine(33);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetryUpdateLine(36);
        robot.robotHardwareMapInit(hardwareMap);
        telemetryUpdateLine(38);
        robot.driveInit();
        telemetryUpdateLine(40);
        slideServo = hardwareMap.get(CRServo.class, "slideservo");
        slide = hardwareMap.get(DcMotor.class, "slide");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        telemetryUpdateLine(44);
        spinner.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetryUpdateLine(46);
        // We want to turn off velocity control for auto
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetryUpdateLine(50);
        Trajectory t1 = drive.trajectoryBuilder(new Pose2d())
                .forward(7)
                .build();
        Trajectory t2 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(6)
                .build();
        Trajectory t3 = drive.trajectoryBuilder(new Pose2d())
                .back(3)
                .build();
        Trajectory t4 = drive.trajectoryBuilder(new Pose2d())
                .forward(24)
                .build();
        telemetryUpdateLine(63);
        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);
        telemetryUpdateLine(67);

        waitForStart();
        boolean stop = false;
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested() && !stop) {
            drive.followTrajectory(t1);
            sleep(1000);
            drive.followTrajectory(t2);
            sleep(1000);
            drive.followTrajectory(t3);
            sleep(1000);
            spinner.setPower(1);
            sleep(4000);
            spinner.setPower(0);
            drive.turn(-35);
            drive.followTrajectory(t4);
            sleep(2000);
            slide.setPower(1);
            sleep(1500);
            slide.setPower(0);
            slideServo.setPower(1);
            sleep(1000);
            slideServo.setPower(0);
            drive.turn(30);
            sleep(1000);
            drive.followTrajectory(t1);
            sleep(10000);
            stop = true;
           stop();
        }
    }
    public void telemetryUpdateLine(int line) {
        telemetry.addData("Status", "Line: " + line);
        telemetry.update();
        sleep(500);
    }
}
