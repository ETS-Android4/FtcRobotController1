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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 *                     TeleOP Controls
 *       _=====_                               _=====_
 *      / _____ \                             / _____ \
 *    /.-'_____'-.---------------------------.-'_____'-\
 *   /   |     |  '.        L O G I        .'  |     |   \
 *  / ___| /|\ |___ \                     / ___| (Y) |___ \
 * / |      |      | ;  __           _   ; |             | ;
 * | | <---   ---> | | |__|         |_|  | |(X)       (B)| |
 * | |___   |   ___| ; SELECT      START ; |___       ___| ;
 * |\    | \|/ |    /  _     ___      _   \    | (A) |    /|
 * | \   |_____|  .','" "', |___|  ,'" "', '.  |_____|  .' |
 * |  '-.______.-' /       \ANALOG/       \  '-._____.-'   |
 * |               |       |------|       |                |
 * |              /\       /      \       /\               |
 * |             /  '.___.'        '.___.'  \              |
 * |            /                            \             |
 *  \          /                              \           /
 *   \________/                                \_________/
 *                         Gamepad 1:
 *
 * gamepad1.dpad_up :
 * gamepad1.dpad_left :
 * gamepad1.dpad_right :
 * gamepad1.dpad_down :
 * gamepad1.a :
 * gamepad1.x : Duck
 * gamepad1.b :
 * gamepad1.y :
 * gamepad1.left_trigger :
 * gamepad1.left_bumper :
 * gamepad1.right_trigger :
 * gamepad1.right_bumper :
 * gamepad1.left_stick_y :
 * gamepad1.left_stick_x :
 * gamepad1.right_stick_y :
 * gamepad1.right_stick_x :
 * gamepad1.left_stick_button :
 * gamepad1.right_stick_button :
 * gamepad1.start :
 * gamepad1.back :
 *
 *                      TeleOP Controls
 *        _=====_                               _=====_
 *       / _____ \                             / _____ \
 *     /.-'_____'-.---------------------------.-'_____'-\
 *    /   |     |  '.        L O G I        .'  |     |   \
 *   / ___| /|\ |___ \                     / ___| (Y) |___ \
 *  / |      |      | ;  __           _   ; |             | ;
 *  | | <---   ---> | | |__|         |_|  | |(X)       (B)| |
 *  | |___   |   ___| ; SELECT      START ; |___       ___| ;
 *  |\    | \|/ |    /  _     ___      _   \    | (A) |    /|
 *  | \   |_____|  .','" "', |___|  ,'" "', '.  |_____|  .' |
 *  |  '-.______.-' /       \ANALOG/       \  '-._____.-'   |
 *  |               |       |------|       |                |
 *  |              /\       /      \       /\               |
 *  |             /  '.___.'        '.___.'  \              |
 *  |            /                            \             |
 *   \          /                              \           /
 *    \________/                                \_________/
 *                          Gamepad 2
 *
 * gamepad2.dpad_up :
 * gamepad2.dpad_left :
 * gamepad2.dpad_right :
 * gamepad2.dpad_down :
 * gamepad2.a :
 * gamepad2.x :
 * gamepad2.b :
 * gamepad2.y :
 * gamepad2.left_trigger :
 * gamepad2.left_bumper :
 * gamepad2.right_trigger :
 * gamepad2.right_bumper :
 * gamepad2.left_stick_y :
 * gamepad2.left_stick_x :
 * gamepad2.right_stick_y :
 * gamepad2.right_stick_x :
 * gamepad2.left_stick_button :
 * gamepad2.right_stick_button :
 * gamepad2.start :
 * gamepad2.back :
 */
/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omnidirectional OpMode", group="Iterative Opmode")
//@Disabled
public class BasicOpMode_Omnidirectional extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private CRServo slideServo = null;
    private DcMotor intake = null;
    private DcMotor DCslide = null;
    private DcMotor spinner = null;
    double speedMultiplier = 1;
    double GP2speedMultiplier = 1;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Declare our motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        DCslide = hardwareMap.get(DcMotor.class, "slide");
        slideServo = hardwareMap.get(CRServo.class, "slideServo");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        slideServo.setDirection(CRServo.Direction.FORWARD);
        spinner.setDirection(DcMotor.Direction.FORWARD);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        // Setup a variable for each drive wh

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;


        // Set a multiplier for the speed to use when the b button is pressed

        speedMultiplier = Range.clip((1.25 - gamepad1.left_trigger), 0, 1);
        GP2speedMultiplier = Range.clip((1.25 - gamepad2.left_trigger), 0, 1);

        // Send calculated power to wheels
        if (gamepad1.y) {
            leftBackDrive.setPower(1 * speedMultiplier);
            rightBackDrive.setPower(1 * speedMultiplier);
            leftFrontDrive.setPower(-1 * speedMultiplier);
            rightFrontDrive.setPower(-1 * speedMultiplier);
        } else if(gamepad1.a) {
            leftBackDrive.setPower(-1 * speedMultiplier);
            rightBackDrive.setPower(-1 * speedMultiplier);
            leftFrontDrive.setPower(1 * speedMultiplier);
            rightFrontDrive.setPower(1 * speedMultiplier);
        } else {
            leftBackDrive.setPower(Range.clip((gamepad1.left_stick_x - (gamepad1.right_stick_y * 0.7) - gamepad1.right_stick_x), -1, 1) * speedMultiplier);
            rightBackDrive.setPower(Range.clip((-gamepad1.left_stick_x - (gamepad1.right_stick_y) + gamepad1.right_stick_x), -1, 1) * speedMultiplier);
            leftFrontDrive.setPower(Range.clip((gamepad1.left_stick_x - (gamepad1.right_stick_y * 0.7) + (gamepad1.right_stick_x * 0.4)), -1, 1) * speedMultiplier);
            rightFrontDrive.setPower(Range.clip((-gamepad1.left_stick_x - (gamepad1.right_stick_y) - (gamepad1.right_stick_x * 0.4)), -1, 1) * speedMultiplier);
        }


        //Duck Spinner
        if (gamepad1.x) {
            spinner.setPower(1 * speedMultiplier);
        } else if (gamepad1.b) {
            spinner.setPower(-1 * speedMultiplier);
        } else {
            spinner.setPower(0);
        }


        //Movement for the servo to drop off cubes/spheres
        if (gamepad2.left_bumper) {
            slideServo.setPower(0.5 * GP2speedMultiplier);
            telemetry.addData("Status", slideServo.getPower());
        } else if (gamepad2.right_bumper) {
            slideServo.setPower(-0.5 * GP2speedMultiplier);
            telemetry.addData("Status", slideServo.getPower());
        } else {
            slideServo.setPower(0);
        }

        //Intake
        if (gamepad2.x) {
            intake.setPower(-1 * GP2speedMultiplier);
        } else if (gamepad2.b) {
            intake.setPower(1 * GP2speedMultiplier);
        } else {
            intake.setPower(0);
        }

        //Slide
        if (gamepad2.y) {
            DCslide.setPower(-1 * GP2speedMultiplier);
        } else if (gamepad2.a) {
            DCslide.setPower(1 * GP2speedMultiplier);
        } else {
            DCslide.setPower(0);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

    }

    //  * Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {
    }
}