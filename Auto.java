
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
    DcMotor liftermotor;
    DcMotor drivefrontone;
    DcMotor drivefronttwo;
    DcMotor drivebackone;
    DcMotor drivebacktwo;
    DcMotor relicthrower;
    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    public void driveForward(float speed) {
        drivefrontone.setPower(-speed);
        drivefronttwo.setPower(speed);
        drivebackone.setPower(-speed);
        drivebacktwo.setPower(speed);
    }
    public void waitFor(float times) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < times/5)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
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
        // jt = hardwareMap.get(Servo.class, "jt");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        // robot.leftDrive.setPower(FORWARD_SPEED);
        // robot.rightDrive.setPower(FORWARD_SPEED);
        driveForward((float) 0.4);
        
        waitFor((float) 1.0);

        // Step 2:  Spin right for 1.3 seconds
        // robot.leftDrive.setPower(TURN_SPEED);
        // robot.rightDrive.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Backwards for 1 Second
        // robot.leftDrive.setPower(-FORWARD_SPEED);
        // robot.rightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
