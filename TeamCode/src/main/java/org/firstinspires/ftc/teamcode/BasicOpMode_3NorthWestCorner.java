/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="3NorthWest")
//@Disabled
public class BasicOpMode_3NorthWestCorner extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    final double     COUNTS_PER_MOTOR_REV    = 11200 ;    // eg: NeveRest40 Motor Encoder
    final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 12.5663706;

    int newRightFrontTarget;
    int newRightBackTarget;
    int newLeftFrontTarget;
    int newLeftBackTarget;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean stop = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() & stop == false) {
            stop_reset_encoders();

            // Send calculated power to wheels

            set_position(5);
            go_to_position();

            telemetry_update();

            leftFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + (int)(10.205 * COUNTS_PER_INCH));
            leftBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + (int)(10.205 * COUNTS_PER_INCH));

            telemetry_update();

            move(1, 3000);

            telemetry_update();

            // Show the elapsed game time and wheel power.
            telemetry_update();
            stop = true;
        }
    }
    public void telemetry_update() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Encoder", "Front Left: " + leftFrontDrive.getCurrentPosition());
        telemetry.addData("Encoder", "Front Right: " + rightFrontDrive.getCurrentPosition());
        telemetry.addData("Encoder", "Back Left: " + leftBackDrive.getCurrentPosition());
        telemetry.addData("Encoder", "Back Right: " + rightBackDrive.getCurrentPosition());
        telemetry.update();
    }

    public void go_to_position(){
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void stop_reset_encoders() {
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void set_position(int position) {
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + (int)(position * COUNTS_PER_INCH));
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + (int)(position * COUNTS_PER_INCH));
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + (int)(position * COUNTS_PER_INCH));
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + (int)(position * COUNTS_PER_INCH));

    }
    public void move(int speed, int time) {
        rightFrontDrive.setPower(speed);
        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        sleep(time);

        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
