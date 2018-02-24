
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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

public class AutoBackLeft180 extends LinearOpMode {

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
    
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
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
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "gs");
        imu.initialize(parameters);
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // telemetry.addD
        while(true)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (angles.firstAngle < 84) {
                
                telemetry.addData("Right", "yes: " + angles.firstAngle);
                telemetry.update();
                drivefrontone.setPower(0.35);
                drivefronttwo.setPower(0.35);
                drivebackone.setPower(0.35);
                drivebacktwo.setPower(0.35);
            }
            else {
                telemetry.addData("Right","no" + angles.firstAngle);
                drivebackone.setPower(0.0);
                drivebacktwo.setPower(0.0);
                drivefrontone.setPower(0.0);
                drivefronttwo.setPower(0.0);
                break;
            }
        }
        
    }
    public void turnLeft() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "gs");
        imu.initialize(parameters);
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
       while(true)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (angles.firstAngle < 84) {
                
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
    }
    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        // robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        // telemetry.addData("Status", "Ready to run");    //
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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters2 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        
        parameters2.vuforiaLicenseKey = "AUbpCEX/////AAAAGTLACtHX+0EJm3l+5ZOENEOBdEOdMJ7zgnhbLJN82nG6pys+khC3Y0l2odX+drpSwnRRzYYNQouYsqEwxCQo+vmM8qQuSR++lngbVq/7tZ+35AAyjKe+HO1NTcB1V9PbjyHtOUYAoPjfhW4/ErAxZ3BC+enW5VnBxmZMPeWVsVuMyDMiBFapkva3CxTZ7dN0mjBCp7AUOCYVSXPKNEjIyahN7pTsJV+zahoF5Gi2n0tM5DK2jRUD4P6HO95DL+G5cNECyC0BZVxdtkUz3upFnH+oYvI3b+QR/1s2o8RnPfE/k/BxirirkT4ADQl5Ct3+b0InnG9CyWydvvr7l/fkfWV79DjbDgKTnocKk250Jgba";
        //gs = hardwareMap.get(GyroSensor.class, "gs");
        // jt = hardwareMap.get(Servo.class, "jt");
        // Wait for the game to start (driver presses PLAY)
        //testGyro();
        // telemetry.addData("THis", "this: " + angles);
        telemetry.update();
         parameters2.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters2);

        
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        telemetry.addData("Waiting", "for three seconds");
        telemetry.update();
        
        sleep(3000);
        relicTrackables.activate();
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
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                
                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.update();
                if(vuMark == RelicRecoveryVuMark.LEFT) {
                    telemetry.addData("LEFT(RUNNING) --> ","%s", vuMark);
                    telemetry.update();
                  grabBlock();
        // waitFor((float)0.5);
        sleep(1000);
        // Lift grabbed block
        liftBlock();
        sleep(4000);
        liftermotor.setPower(0.0);
        driveBack((float)0.5);
        waitFor((float) 1.85);
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
        //waitFor((float) 0.3);
        // Drop block
        sleep(1000);
        dropBlock();
        sleep(1000);
        liftBlock();
        sleep(3000);
        driveBack((float)0.3);
        
        //driveBack((float)0.1);
        waitFor((float) 0.0025);
        bringDown();
        telemetry.addData("Program", "Complete");
        telemetry.update();
        sleep(2000);
                }
                else if(vuMark == RelicRecoveryVuMark.RIGHT) {
                    telemetry.addData("RIGHT(RUNNING) --> ","%s", vuMark);
                    grabBlock();
        // waitFor((float)0.5);
        sleep(1000);
        // Lift grabbed block
        liftBlock();
        sleep(4000);
        liftermotor.setPower(0.0);
        driveBack((float)0.25);
        waitFor((float) 1.80);
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
        //waitFor((float) 0.3);
        // Drop block
        sleep(1000);
        dropBlock();
        sleep(1000);
        liftBlock();
        sleep(3000);
        driveBack((float)0.3);
        
        //driveBack((float)0.1);
        waitFor((float) 0.0025);
        bringDown();
        telemetry.addData("Program", "Complete");
        telemetry.update();
        sleep(2000);
                }
                else if(vuMark == RelicRecoveryVuMark.LEFT) {
                    telemetry.addData("CENTER(RUNNING) --> ","%s", vuMark);
                    grabBlock();
        // waitFor((float)0.5);
        sleep(1000);
        // Lift grabbed block
        liftBlock();
        sleep(4000);
        liftermotor.setPower(0.0);
        driveBack((float)0.374);
        waitFor((float) 1.80);
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
        //waitFor((float) 0.3);
        // Drop block
        sleep(1000);
        dropBlock();
        sleep(1000);
        liftBlock();
        sleep(3000);
        driveBack((float)0.3);
        
        //driveBack((float)0.1);
        waitFor((float) 0.0025);
        bringDown();
        telemetry.addData("Program", "Complete");
        telemetry.update();
        sleep(2000);
                }
                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                // OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                // telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                // if (pose != null) {
                //     VectorF trans = pose.getTranslation();
                //     Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                //     // Extract the X, Y, and Z components of the offset of the target relative to the robot
                //     double tX = trans.get(0);
                //     double tY = trans.get(1);
                //     double tZ = trans.get(2);
                    
                //     // Extract the rotational components of the target relative to the robot
                //     double rX = rot.firstAngle;
                //     double rY = rot.secondAngle;
                //     double rZ = rot.thirdAngle;
                // }
            }
            else {
                telemetry.addData("VuMark", "not visible --> ASSUMING LEFT");
                telemetry.update();
                grabBlock();
        // waitFor((float)0.5);
        sleep(1000);
        // Lift grabbed block
        liftBlock();
        sleep(4000);
        liftermotor.setPower(0.0);
        driveBack((float)0.4);
        waitFor((float) 0.46);
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
        driveForward((float)0.2);
        waitFor((float) 0.2);
        // Drop block
        sleep(1000);
        dropBlock();
        sleep(1000);
        liftBlock();
        sleep(3000);
        driveBack((float)0.3);
        
        //driveBack((float)0.1);
        waitFor((float) 0.25);
        bringDown();
        telemetry.addData("Program", "Complete");
        telemetry.update();
        sleep(2000);
            }

            telemetry.update();
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
    
