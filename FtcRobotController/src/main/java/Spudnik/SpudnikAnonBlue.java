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
//Add speaker, spoiler, mudflaps (and reverse alarm), tinted PlexiGlass
package Spudnik;

import android.app.admin.DelegatedAdminReceiver;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


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

@Autonomous(name="SpudnikAnonBlue", group="Linear Opmode") //auton mode
//@Disabled
public class SpudnikAnonBlue extends LinearOpMode { //class config

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime(); //time
    private DcMotor leftDrive = null; //left motor
    private DcMotor rightDrive = null; //right motor

    private DcMotor duck = null; //duck spinner for carousel
    private DcMotor armRaise = null; //arm raiser
    private Servo clampServo = null; //servo for clamp



    private int stage = 0; //stage for auto


    private BNO055IMU imu = null; //imu declaration

    private Orientation angles; //heading degree variable
    private double curHeading; //numerical heading in double form

    private Acceleration gravity; //acceleration
    private double accX; //numerical acceleration x
    private double accY; //numerical acceleration y

    private Acceleration overall; //overall acceleration
    private double overX; //numerical acceleration overall x
    private double overY; //numerical acceleration overall y

    private Position map; //position of robot on map
    private double mapX; //numerical map position x
    private double mapY; //numerical map position y

    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Conprivate DcMotor raise = null;troller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive"); //motor hardware class
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive"); //motor hardware class
        duck = hardwareMap.get(DcMotor.class, "spin"); //motor hardware class
        clampServo = hardwareMap.get(Servo.class, "clamp"); //servo hardware class
        armRaise = hardwareMap.get(DcMotor.class, "raise"); //servo hardware class


        imu = hardwareMap.get(BNO055IMU.class, "imu"); //imu hardware class
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //make new parameters
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES; //degree is the unit
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; //in meters per second
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true; //log
        parameters.loggingTag          = "IMU"; //logs as IMU (see logcat)
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); //log acceleration
        imu.initialize(parameters); //inialize all paraneters
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); //calibrate the parameters



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE); //left is reversed because it is flipped
        rightDrive.setDirection(DcMotor.Direction.FORWARD); //set direction
        duck.setDirection(DcMotor.Direction.FORWARD); //set direction
        armRaise.setDirection(DcMotorSimple.Direction.FORWARD); //set direction of arm raise




        // Wait for the game to start (driver presses PLAY)
        waitForStart(); //wait for start
        runtime.reset(); //once strated, reset clock

        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) { //while auto is running
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); //reset caliberation
            checkOrientation(); //check the orientation
            checkAcceleration(); //check acceleration
            checkOverallAcceleration(); //check overall acceleration
            checkNavigation(); //check navigation
            // Setup a variable for each drive wheel to save power level for telemetry


            double leftPower= 0; //declare motor power on the left
            double rightPower = 0; //declare motor power on the right
            double spinPower = 0; //power for duck spin
            double raisePower = 0; //declare power for arm pwo
            boolean isClamp = true;


            if(stage == 0) {
                rightPower = 100;
                leftPower = -30;
            }
            if(curHeading >= 70)stage = 1;

            if(stage == 1){
                rightPower = -100;
                leftPower = -100;
                spinPower = 100;
            }



            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            armRaise.setPower(raisePower);
            duck.setPower(spinPower);

            telemetry.addData("Degrees", "* (%.2f)", curHeading);
            telemetry.addData("X - Y Acceleration", "X (%.2f), Y (%2f)", accX, accY);
            telemetry.addData("X - Y Overall", "X (%.2f), Y (%2f)", overX, overY);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Clamp", "Trigger1 (%.2f), Trigger2 (%.2f)", gamepad1.right_trigger, gamepad2.right_trigger);
            telemetry.addData("CarSpin", "Motor (%2f)", spinPower);
            telemetry.addData("ArmRaise", "Motor (%2f)", raisePower);
            telemetry.update();






        }



    }
    public void checkOrientation() {
    // read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        // and save the heading
        curHeading = angles.firstAngle;
    }
    public void checkAcceleration(){
        gravity = this.imu.getAcceleration();
        accX = gravity.xAccel;
        accY = gravity.yAccel;
    }
    public void checkOverallAcceleration(){
        overall = this.imu.getOverallAcceleration();
        overX = overall.xAccel;
        overY = overall.yAccel;
    }
    public void checkNavigation(){
        map = this.imu.getPosition();
        mapX = map.x;
        mapY = map.y;
        telemetry.addData("X - Y Map", "X (%2f), Y (%2f)", mapX, mapY);
    }

}
