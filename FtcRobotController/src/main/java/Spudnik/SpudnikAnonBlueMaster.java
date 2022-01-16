package Spudnik;

import android.app.admin.DelegatedAdminReceiver;
import android.widget.Button;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@Autonomous(name="EX Blue En", group="Linear Opmode") //auto mode
public class SpudnikAnonBlueMaster extends LinearOpMode { //class config


    private final ElapsedTime runtime = new ElapsedTime(); //time
    private DcMotorEx leftDrive = null; //left motor
    private DcMotorEx rightDrive = null; //right motor

    private DcMotor duck = null; //duck spinner for carousel
    private DistanceSensor spinSensor = null;
    private RevTouchSensor duckButton = null;
    private RevTouchSensor button = null;
    private RevColorSensorV3 capSensor = null;
    private Servo duckServo = null;
    private Servo clampServo = null; //servo for clamp

    private DcMotor armRaise = null; //arm raiser


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


    private double leftPower= 0; //declare motor power on the left
    private double rightPower = 0; //declare motor power on the right
    private double spinPower = 0; //power for duck spin
    private double raisePower = 0; //declare power for arm pwo
    private double numSen = 0;
    private boolean isClamp = true; //set clamp to true
    private boolean firstrun = true;

    @Override
    public void runOpMode() {
        initEverything();
        while (opModeIsActive()) { //while auto is running
            getTelemetry(); //gets the telemetry
            getNumbers(); // gets the number for hte teltery
            goSomewhere(500); //goes to forward by a little
            turnLeft(90); //turns left 90 degrees
            goSomewhere(-800); //goes backwards into the duck spinner
            duck.setPower(1); //turn on the duck spinner
            sleep(4000); //sleeps for 4 seconds, allows duck to fall off
            duck.setPower(0); //turn off duck spinner
            goSomewhere(3000); //go forwards
            armRaise.setPower(1); //raise arm
            sleep(1000); //sleep for 1 second to allow arm to raise
            armRaise.setPower(0); //turn of power to arm
            goSomewhere(300); //go forward
            goSomewhere(300);

            //turn off all the encoders, so it doesn't mess up the encoders
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


           getTelemetry(); //get the telemetry
            break; //halt the program


        }

    }


    public void turnRight(int input){
        resetEncoders(input);

        leftDrive.setMotorEnable();
        rightDrive.setMotorEnable();
        leftDrive.setTargetPosition(input);
        rightDrive.setTargetPosition(-input);
        leftDrive.setVelocity(2000);
        rightDrive.setVelocity(-2000);

        while(leftDrive.isBusy() || rightDrive.isBusy()){
            getTelemetry();
            getNumbers();
        }

    }
    public void turnLeft(int degrees){
        resetEncoders(0);

        leftDrive.setMotorEnable();
        rightDrive.setMotorEnable();
        leftDrive.setTargetPosition(-10000);
        rightDrive.setTargetPosition(10000);
        leftDrive.setVelocity(-2000);
        rightDrive.setVelocity(2000);

        while(leftDrive.isBusy() || rightDrive.isBusy()){
            getTelemetry();
            getNumbers();
            if(curHeading >= degrees - 20) {
                leftDrive.setVelocity(-1000);
                rightDrive.setVelocity(1000);
            }
            if(curHeading >= degrees - 10) {
                leftDrive.setVelocity(-500);
                rightDrive.setVelocity(500);
            }
            if(curHeading >= degrees - 5){
                leftDrive.setVelocity(0);
                rightDrive.setVelocity(0);
                break;
            }
        }
    }
    public void goSomewhere(int input){
        resetEncoders(input);

        if(input < 0) {
            leftDrive.setVelocity(-2000);
            rightDrive.setVelocity(-2000);
        }
        else{
            leftDrive.setVelocity(2000);
            rightDrive.setVelocity(2000);
        }
        while(roughBusy(50)){
            getTelemetry();
            getNumbers();
        }

    }

    public void resetEncoders(int input){
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setTargetPosition(input);
        rightDrive.setTargetPosition(input);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void getNumbers(){
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); //reset calibration
        checkOrientation(); //check the orientation
        checkAcceleration(); //check acceleration/home/william/AndroidStudioProjects/6dayschedule/home/william/AndroidStudioProjects/6dayschedule
        checkOverallAcceleration(); //check overall acceleration
        checkNavigation(); //check navigation
        spinPower = 0; //power for duck spin
        raisePower = 0; //declare power for arm pwo
        numSen = spinSensor.getDistance(DistanceUnit.CM);
    }
    public boolean roughBusy(int howOff){

        if(leftDrive.getCurrentPosition() - leftDrive.getTargetPosition() <= howOff && leftDrive.getCurrentPosition() - leftDrive.getTargetPosition() >= -howOff) return true;
        if(rightDrive.getCurrentPosition() - rightDrive.getTargetPosition() <= howOff && rightDrive.getCurrentPosition() - rightDrive.getTargetPosition() >= -howOff) return true;
        else return false;

    }
    public void initEverything(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive"); //motor hardware class
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setTargetPosition(0);

        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive"); //motor hardware class
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setTargetPosition(0);


        duck = hardwareMap.get(DcMotor.class, "spin"); //motor hardware class
        clampServo = hardwareMap.get(Servo.class, "clamp"); //servo hardware class
        armRaise = hardwareMap.get(DcMotor.class, "raise"); //servo hardware class
        spinSensor = hardwareMap.get(DistanceSensor.class, "spinSensor");
        duckButton = hardwareMap.get(RevTouchSensor.class, "duckButton");
        duckServo = hardwareMap.get(Servo.class, "duckServo");
        button = hardwareMap.get(RevTouchSensor.class, "button");

        imu = hardwareMap.get(BNO055IMU.class, "imu"); //imu hardware class
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //make new parameters
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES; //degree is the unit
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; //in meters per second
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true; //logInput
        parameters.loggingTag          = "IMU"; //logs as IMU (see logcat)
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); //log acceleration
        imu.initialize(parameters); //initialize all parameters
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); //calibrate the parameters



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        leftDrive.setDirection(DcMotor.Direction.REVERSE); //left is reversed because it is flipped
        rightDrive.setDirection(DcMotor.Direction.FORWARD); //set direction
        duck.setDirection(DcMotor.Direction.FORWARD); //set direction
        armRaise.setDirection(DcMotorSimple.Direction.FORWARD); //set direction of arm raise




        // Wait for the game to start (driver presses PLAY)
        waitForStart(); //wait for start
        runtime.reset(); //once started, reset clock

        // run until the end of the match (driver presses STOP)
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

    public void getTelemetry(){
        telemetry.addData("Degrees", "* (%.2f)", curHeading); //degrees telemetry
        telemetry.addData("X - Y Acceleration", "X (%.2f), Y (%2f)", accX, accY); //acceleration telemetry
        telemetry.addData("X - Y Overall", "X (%.2f), Y (%2f)", overX, overY); //overall acceleration telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString()); //run time telemetry
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower); // wheel telemetry
        telemetry.addData("Clamp", "Trigger1 (%.2f), Trigger2 (%.2f)", gamepad1.right_trigger, gamepad2.right_trigger); //servo clamp telemetry
        telemetry.addData("DuckSpin", "Motor (%2f)", spinPower); //duck spinner motor telemetry
        telemetry.addData("ArmRaise", "Motor (%2f)", raisePower); //arm motor telemetry
        telemetry.addData("stage", stage);
        telemetry.addData("position", leftDrive.getCurrentPosition());
        telemetry.addData("position", rightDrive.getCurrentPosition());
        telemetry.addData("isBusy", leftDrive.isBusy());
        telemetry.update(); //update

    }

}
