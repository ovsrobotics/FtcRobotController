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

package Spudnik;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.bb
 *        Integrate manipulators on robot.
 Programming to add code to enable the manipulators.
 Complete Control Award paperwork (if applicable).
 Experiment and learn about sensors.
 Rehearse presentation for judged awards at events.
 Review the Preparing for a Competition.

 Week 6 – Meet Zero Week – OCT 23-30

 Improve manipulators.
 Determine how sensors can be used on the robot to help improve performance.
 Review the Pre-Match Checklist.

 Week 7 – OCT 31-NOV 5

 Practice using manipulators with game elements.
 Adjust manipulators.
 Continue to improve robot (may require more parts).


 Week 8 – Meet 1 – NOV 6-13

 Improve programming.
 Prep and practice for competition.
 Develop a competition checklist.

 Week 9 – NOV 14-19  (Then go on Thanksgiving Break)

 Tune-up robot for next competition.
 Schedule outreach activity.

 Week 10 – NOV 29 – DEC 04

 Competition Season Week 11
 Competition Season

 Week 11 – DEC 05-11 - Meet 2

 Improve all you can.

 Week 12 – JAN 10-15

 Improve all you can.

 Week 13 – JAN 16-22 – Meet 3

 Sign up for Qualifier, WHICH ONE? We can choose
 Schedule Dean’s List nominations for the 15th of the month prior to an event.
 Sign up for Qualifier, WHICH ONE? We can choose
 Schedule Dean’s List nominations for the 15th of the month prior to an event.

 Week 14 – Qualifier Tournament

 Prep for State/Region Championships (if applicable)
 Develop videos for Compass and Promote award
 Start fundraising for World Championship (if applicable)



 Progamming Resources

 Android Studio URL
 JAVA SDK URL
 WINDOWS USERS: Java installation instructions URL
 programming re
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="EMAN", group="Linear Opmode")
//@Disabled
public class EMAN extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private DcMotor duck = null;
    private DcMotor armRaise = null;
    private Servo clampServo = null;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Conprivate DcMotor raise = null;troller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        duck = hardwareMap.get(DcMotor.class, "spin");
        clampServo = hardwareMap.get(Servo.class, "clamp");
        armRaise = hardwareMap.get(DcMotor.class, "raise");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        duck.setDirection(DcMotor.Direction.FORWARD);
        armRaise.setDirection(DcMotorSimple.Direction.FORWARD);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double spinPower; //power for duck spin
            double raisePower;
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            boolean isClamp = true;





            if(gamepad1.a || gamepad2.a) spinPower = 100; //if a button is pressed, sets power to spin 100
            else if(gamepad1.y || gamepad2.y) spinPower = -100;
            else spinPower = 0; //not pressing, power is off


            if(gamepad2.right_trigger >= .1 || gamepad1.right_trigger >= .1 ) isClamp = false;
            else isClamp = true;

            if(isClamp) clampServo.setPosition(1); //sets position of servo
            else clampServo.setPosition(0);

            if(gamepad1.dpad_up || gamepad2.dpad_up) raisePower = 50;
            else if(gamepad1.dpad_down || gamepad2.dpad_down)raisePower = -50;
            else raisePower = 0;

            armRaise.setPower(raisePower);
            duck.setPower(spinPower);

            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            //leftPower  = -gamepad1.left_stick_y ;
            //rightPower = -gamepad1.right_stick_y ;
            // Send calculated power to wheels

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            // Show the elapsed HADPOWER (%.2f"), spinPower )game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Clamp", "Trigger1 (%.2f), Trigger2 (%.2f)", gamepad1.right_trigger, gamepad2.right_trigger);
            telemetry.addData("CarSpin", "Motor (%2f)", spinPower);
            telemetry.addData("ArmRaise", "Motor (%2f)", raisePower);
            telemetry.update();
        }
    }
}
