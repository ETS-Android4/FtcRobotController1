package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;




@TeleOp
public class BasicOpMode_TeleOP2 extends LinearOpMode{

    //Motors


    //Basic Mecanum drive for joysticks
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        DcMotor leftFrontDrive = hardwareMap.dcMotor.get("leftFrontDrive");
        DcMotor leftBackDrive = hardwareMap.dcMotor.get("leftBackDrive");
        DcMotor rightFrontDrive = hardwareMap.dcMotor.get("rightFrontDrive");
        DcMotor rightBackDrive = hardwareMap.dcMotor.get("rightBackDrive");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor slide = hardwareMap.dcMotor.get("slide");
        CRServo slideServo = hardwareMap.crservo.get("slideServo");

        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        // Make sure your ID's match your configuration






        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double vertical = 0.7*gamepad1.left_stick_y;
            double horizontal = -0.7*gamepad1.left_stick_x;
            double pivot = -0.7*gamepad1.right_stick_x;

            rightFrontDrive.setPower(vertical - pivot - horizontal);
            rightBackDrive.setPower(vertical - pivot + horizontal);
            leftFrontDrive.setPower(vertical + pivot + horizontal);
            leftBackDrive.setPower(vertical + pivot - horizontal);

            //Engage Linear Slide
            if (gamepad1.a) {
                slide.setPower(0.5);
            } else {
                slide.setPower(0);
            }
            if (gamepad1.b) {
                slide.setPower(-0.5);
            } else {
                slide.setPower(0);
            }

            //Intake
            if (gamepad1.x) {
                intake.setPower(-1);
            }
            if (true) {
                slideServo.setPower(gamepad1.right_trigger);

            }
            if (true) {
                slideServo.setPower(-gamepad1.left_trigger);
            }


        }
    }

}