package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
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
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

@Autonomous

public class AFL extends LinearOpMode{
    /*********************************************
    *
    *  Main variable declarations start
    *
    ***********************************************/
    private ElapsedTime     runtime = new ElapsedTime();
    Servo servoRB;
    Servo servoLB;
    Servo servoRF;
    Servo servoLF;
    Servo servoJT;
    GyroSensor gs;
    DcMotor liftermotor;
    DcMotor drivefrontone;
    DcMotor drivefronttwo;
    DcMotor drivebackone;
    DcMotor drivebacktwo;
    DcMotor relicthrower;
    Orientation angles;
    ColorSensor sensorColor;
    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    /*********************************************
    *
    *  End variable declarations
    *
    ***********************************************/
    /*********************************************
    *
    *  Vuforia Initialization and Stuff goes here.
    *
    ***********************************************/
    VuforiaLocalizer vuforia;
    /*********************************************
    *
    *  End Vuforia Stuff
    *
    ***********************************************/
    /*********************************************
    *
    *  Start main methods
    *
    ***********************************************/
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
        liftermotor.setPower(0.9);
        sleep(500);
        liftermotor.setPower(0.0);
        
    }
    public void grabBlock() {
        servoRB.setPosition(0);
        servoRF.setPosition(0);
        servoLB.setPosition(0.4);
        servoLF.setPosition(0.4);
        telemetry.addData("Picked Up", "Block");
        telemetry.update();
    }
    public void dropBlock() {
        servoRB.setPosition(1);
        servoRF.setPosition(1);
        servoLB.setPosition(1);
        servoLF.setPosition(1);
        telemetry.addData("Dropped", "Block");
        telemetry.update();
    }
    public void bringDown() {
        liftermotor.setPower(-0.5);
        sleep(500);
        liftermotor.setPower(0.0);
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
    public void sleepy(int time22) {
        sleep(time22*1000);
    }
    public void ballhitterdown()
    {
        servoJT.setPosition(0.6);
    }
    public void ballhitterup()
    {
        
        servoJT.setPosition(0.43);
    }
    
    /*********************************************
    *
    *  End Methods
    *
    ***********************************************/
    
    
    @Override
    public void runOpMode() {

        /*********************************************
        *
        *  Start Declarations and variables
        *
        ***********************************************/

        String ballDetected = "none";
        String vuString = "unknown";
        telemetry.update();
        servoRB = hardwareMap.get(Servo.class, "rb");
        servoRF = hardwareMap.get(Servo.class, "rt");
        servoLB = hardwareMap.get(Servo.class, "lb");
        servoLF = hardwareMap.get(Servo.class, "lt");
        servoRB.setDirection(Servo.Direction.REVERSE);
        
        servoLF.setDirection(Servo.Direction.REVERSE);
        servoJT = hardwareMap.get(Servo.class, "jt");
        /*********************************************
        *
        *  Start Color Sensor Init
        *
        ***********************************************/
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        /*********************************************
        *
        *  End Color Sensor Init
        *
        ***********************************************/
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
        parameters2.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters2);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        /*********************************************
        *
        *  End Declarations and variables
        *
        ***********************************************/
        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        /*********************************************
        *
        *  Main Program start
        *
        ***********************************************/
        waitForStart();
        relicTrackables.activate();
        grabBlock();
        sleep(500);
        liftBlock();
        
        sleep(1500);
        
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
            if(vuMark == RelicRecoveryVuMark.RIGHT) {
                vuString = "right";
            } else if(vuMark == RelicRecoveryVuMark.LEFT) {
                vuString = "left";
            } else if(vuMark == RelicRecoveryVuMark.CENTER) {
                vuString = "center";
            }
        } else {
            telemetry.addData("VuMark", "not visible --> ASSUMING CENTER");
            telemetry.update();
        }
        /*********************************************
        *
        *  Start Jewel arm code
        *
        ***********************************************/
        ballhitterdown();
        telemetry.addData("Waiting", "for balldropper -- 0.5 sec");
        telemetry.update();
        sleep(4000);
        telemetry.addData("Looking", "for ball");
        telemetry.update();
        //  Get color of ball:
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
        if(hsvValues[0] > 100 && hsvValues[0] < 319)
        {
            telemetry.addData("Blue Ball Detected ", hsvValues[0]);
            telemetry.update();
            ballDetected = "blue";
        } else if (hsvValues[0] > 327 || hsvValues[0] < 20) {
            telemetry.addData("Red Ball Detected ", hsvValues[0]);
            telemetry.update();
            ballDetected = "red";
        } else {
            telemetry.addData("No Ball Detected ", hsvValues[0]);
            telemetry.update();
        }
            
        if(ballDetected == "blue")
        {
            telemetry.addData("Blue Ball Loop started", hsvValues[0]);
            telemetry.update();
            while(true) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (angles.firstAngle < 7) {
                    drivefrontone.setPower(-0.35);
                    drivefronttwo.setPower(-0.35);
                    drivebackone.setPower(-0.35);
                    drivebacktwo.setPower(-0.35);
                }
                else {
                    // telemetry.addData("Left","no" + angles.firstAngle);
                    drivebackone.setPower(0.0);
                    drivebacktwo.setPower(0.0);
                    drivefrontone.setPower(0.0);
                    drivefronttwo.setPower(0.0);
                    break;
                }
            
            }
            ballhitterup();
            sleep(3000);
            // telemetry.addData("Attempting to move back..", "..");
            telemetry.update();
            drivefrontone.setPower(0.35);
            drivefronttwo.setPower(0.35);
            drivebackone.setPower(0.35);
            drivebacktwo.setPower(0.35);
            sleep(210);
            drivefrontone.setPower(0.0);
            drivefronttwo.setPower(0.0);
            drivebackone.setPower(0.0);
            drivebacktwo.setPower(0.0);
        } 
        else if(ballDetected == "red")
        {
            telemetry.addData("The robot should hit the BLUE ball--Hue ", hsvValues[0]);
            telemetry.update();
            
               drivefrontone.setPower(0.35);
            drivefronttwo.setPower(0.35);
            drivebackone.setPower(0.35);
            drivebacktwo.setPower(0.35);
            sleep(200);
            drivefrontone.setPower(0.0);
            drivefronttwo.setPower(0.0);
            drivebackone.setPower(0.0);
            drivebacktwo.setPower(0.0); 
            sleep(1500);
            ballhitterup();
            sleep(1000);
            telemetry.addData("Attempting to move back", "..");
            telemetry.update();
            sleep(500);
            drivefrontone.setPower(-0.35);
            drivefronttwo.setPower(-0.35);
            drivebackone.setPower(-0.35);
            drivebacktwo.setPower(-0.35);
            sleep(201);
            drivefrontone.setPower(0.0);
            drivefronttwo.setPower(0.0);
            drivebackone.setPower(0.0);
            drivebacktwo.setPower(0.0);
        } 
        else {
            ballhitterup();
            sleep(1500);
        }
        /*********************************************
        *
        *  End Jewel arm code
        *
        ***********************************************/
        /*********************************************
        *
        *  Start Vuforia code
        *
        ***********************************************/
        telemetry.addData("Waiting", "for 1.5 seconds -- looking for VuMark");
        telemetry.update();
        
        liftermotor.setPower(0.0);
        if(vuString == "unknown" || vuString == "center") {
            // driveForward((float)0.5);
            // waitFor((float) 1.5);
            driveForward((float) 0.7);
            sleep(1350);
            drivebackone.setPower(0.0);
            drivebacktwo.setPower(0.0);
            drivefrontone.setPower(0.0);
            drivefronttwo.setPower(0.0);
            
        } else if(vuString == "right") {
            driveForward((float) 0.7);
            sleep(1600);
            drivebackone.setPower(0.0);
            drivebacktwo.setPower(0.0);
            drivefrontone.setPower(0.0);
            drivefronttwo.setPower(0.0);
            // driveForward((float)0.6);
            // waitFor((float) 1.5);
        } else if(vuString == "left") {
            // driveForward((float)0.43);
            // waitFor((float) 1.5);
            driveForward((float) 0.6);
            sleep(1200);
            drivebackone.setPower(0.0);
            drivebacktwo.setPower(0.0);
            drivefrontone.setPower(0.0);
            drivefronttwo.setPower(0.0);
        }
        /*********************************************
        *
        *  End Vuforia code
        *
        ***********************************************/
        sleep(500);
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
        driveForward((float)0.22);
        waitFor((float)0.3);
        sleep(1000);
        dropBlock();
        sleep(1000);
        driveBack((float)0.12);
        waitFor((float) 0.001);
        bringDown();
        sleep(500);
        // grabBlock();
        driveForward((float) 0.7);
        sleep(500);
        dropBlock();
        driveBack((float) 0.3);
        sleep(400);
        drivebackone.setPower(0.0);
                drivebacktwo.setPower(0.0);
                drivefrontone.setPower(0.0);
                drivefronttwo.setPower(0.0);
        telemetry.addData("Program", "Complete");
        telemetry.update();
        /*********************************************
        *
        *  End Main Program
        *
        ***********************************************/
    }
}