
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class Auto extends LinearOpMode {

    /* Declare OpMode members. */
    // HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    Servo servoRB;
    Servo servoLB;
    Servo servoRF;
    Servo servoLF;
    GyroSensor gs;
    DcMotor liftermotor;
    DcMotor drivefrontone;
    DcMotor drivefronttwo;
    DcMotor drivebackone;
    DcMotor drivebacktwo;
    DcMotor relicthrower;
    Orientation angles;
    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    public void driveForward(float speed) {
        drivefrontone.setPower(-speed);
        drivefronttwo.setPower(speed);
        drivebackone.setPower(-speed);
        drivebacktwo.setPower(speed);
    }
    public void driveBack(float speed) {
         drivefrontone.setPower(speed);
        drivefronttwo.setPower(-speed);
        drivebackone.setPower(speed);
        drivebacktwo.setPower(-speed);
    }
    public void waitFor(float times) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < times/3)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    public void liftBlock() {
        liftermotor.setPower(0.2);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.0001)) {
            telemetry.addData("Path", "Lifting: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        
    }
    public void grabBlock() {
        servoRB.setPosition(1.45);
        servoRF.setPosition(1.45);
        servoLB.setPosition(1.45);
        servoLF.setPosition(1.45);
    }
    public void dropBlock() {
        servoRB.setPosition(0.0077);
        servoRF.setPosition(0.25);
        servoLB.setPosition(0.01);
        servoLF.setPosition(0.4);
        telemetry.addData("Grabbed", "Block");
    }
    public void bringDown() {
        liftermotor.setPower(-0.1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.0001)) {
            telemetry.addData("Path", "Dropping Block: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    public void turnRight() {
        // telemetry.addD
        drivefrontone.setPower(0.35);
        drivefronttwo.setPower(0.35);
        drivebackone.setPower(0.35);
        drivebacktwo.setPower(0.35);
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Turning Left: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        
    }
    public void turnLeft() {
        while(angles.firstAngle <= 90)
        {
            drivefrontone.setPower(-0.35);
            drivefronttwo.setPower(-0.35);
            drivebackone.setPower(-0.35);
            drivebacktwo.setPower(-0.35);
            break;
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Turning Left: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        // robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        servoRB = hardwareMap.get(Servo.class, "rb");
        servoRF = hardwareMap.get(Servo.class, "rt");
        servoLB = hardwareMap.get(Servo.class, "lb");
        servoLF = hardwareMap.get(Servo.class, "lt");
        servoRB.setDirection(Servo.Direction.REVERSE);
        servoRF.setDirection(Servo.Direction.REVERSE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "gs");
        imu.initialize(parameters);
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        liftermotor = hardwareMap.dcMotor.get("liftermotor");
        liftermotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicthrower = hardwareMap.dcMotor.get("rrc");
        relicthrower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivefrontone = hardwareMap.dcMotor.get("rf");
        drivefronttwo = hardwareMap.dcMotor.get("lf");
        drivebackone = hardwareMap.dcMotor.get("rba");
        drivebacktwo = hardwareMap.dcMotor.get("lba");
        drivefrontone.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivefronttwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivebackone.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivebacktwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //gs = hardwareMap.get(GyroSensor.class, "gs");
        // jt = hardwareMap.get(Servo.class, "jt");
        // Wait for the game to start (driver presses PLAY)
        //testGyro();
        telemetry.addData("THis", "this: " + angles);
        telemetry.update();
        waitForStart();
        // int thigstuff = 0;
        // while (thigstuff <= 20) {
        //     angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //     AngleUnit ds = AngleUnit.DEGREES;
        //     telemetry.addData("THis", "this: " + angles.firstAngle);
            
        //     telemetry.update();
        //     thigstuff = thigstuff++;
        // };
        
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        // robot.leftDrive.setPower(FORWARD_SPEED);
        // robot.rightDrive.setPower(FORWARD_SPEED);
// Deive off platform
        // driveForward((float) 0.9);
        
        // waitFor((float) 1.75);
        
        // Turn left and go Forward
        // turnLeft();

        
        // driveForward((float) 0.9);
        
        // waitFor((float) 0.5);
        
        // grabBlock();
        grabBlock();
        // waitFor((float)0.5);
        sleep(1000);
        // Lift grabbed block
        liftBlock();
        sleep(4000);
        liftermotor.setPower(0.0);
        driveBack((float)0.374);
        waitFor((float) 1.74);
        sleep(1000);
        
        while(true)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (angles.firstAngle < 88) {
                
                telemetry.addData("Left", "yes: " + angles.firstAngle);
                telemetry.update();
                drivefrontone.setPower(-0.35);
                drivefronttwo.setPower(-0.35);
                drivebackone.setPower(-0.35);
                drivebacktwo.setPower(-0.35);
            }
            else {
                telemetry.addData("Left","no" + angles.firstAngle);
                drivebackone.setPower(0.0);
                drivebacktwo.setPower(0.0);
                drivefrontone.setPower(0.0);
                drivefronttwo.setPower(0.0);
                break;
            }
        }
        // bringDown();
        // driveBack((float) 0.5);
        // waitFor((float) 1.0);
        driveForward((float)0.4225);
        waitFor((float) 0.3);
        // Drop block
        dropBlock();
        sleep(1000);
        
        driveBack((float)0.075);
        liftBlock();
        sleep(3000);
        driveBack((float)0.1);
        waitFor((float) 0.001);
        bringDown();
        sleep(2000);
        driveBack((float)0.025);
        sleep(3000);
        while(true)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (angles.firstAngle < 180) {
                
                telemetry.addData("Left", "yes: " + angles.firstAngle);
                telemetry.update();
                drivefrontone.setPower(-0.35);
                drivefronttwo.setPower(-0.35);
                drivebackone.setPower(-0.35);
                drivebacktwo.setPower(-0.35);
            }
            else {
                telemetry.addData("Left","no" + angles.firstAngle);
                drivebackone.setPower(0.0);
                drivebacktwo.setPower(0.0);
                drivefrontone.setPower(0.0);
                drivefronttwo.setPower(0.0);
                break;
            }
        }
        sleep(1000);
        
        
        // Turn 180 degrees
        // turnRight();
        // turnRight();

        telemetry.addData("Program", "Complete");
        telemetry.update();
        sleep(2000);
    }
}
