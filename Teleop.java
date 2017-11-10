package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp



//@Disabled
public class Teleop extends OpMode {
    
    //Declaring variables
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
   
   // Assignments for lift, however UNUSED CODE
    final double LIFTERMOTORUP      = 0.5;                            // sets rate to move servo
    final double LIFTERMOTORDOWN      = -0.5;
    

    @Override
    public void init() {


        
        servoRB = hardwareMap.get(Servo.class, "rb");
        servoRF = hardwareMap.get(Servo.class, "rt");
        servoLB = hardwareMap.get(Servo.class, "lb");
        servoLF = hardwareMap.get(Servo.class, "lt");
        servoRB.setDirection(Servo.Direction.REVERSE);
        servoRF.setDirection(Servo.Direction.REVERSE);
        
        liftermotor = hardwareMap.dcMotor.get("liftermotor");
        relicthrower = hardwareMap.dcMotor.get("rrc");
        relicthrower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drivefrontone = hardwareMap.dcMotor.get("rf");
        drivefronttwo = hardwareMap.dcMotor.get("lf");
        drivebackone = hardwareMap.dcMotor.get("rba");
        drivebacktwo = hardwareMap.dcMotor.get("lba");


    }
    @Override
    public void loop() {
       
        float rightone = gamepad1.right_stick_y;
        float leftone = -gamepad1.left_stick_y;
        float right2 = -gamepad1.right_trigger;
        float left2 = gamepad1.left_trigger;
        float relicpower = gamepad2.right_stick_y;
        
        float var = (float) 1.5;
        float var2 = (float) -1.5;
        
         //clip the right/left values so that the values never exceed +/- 1
         
        // Creating motor power ranges for driving and relic thrower 
        right2 = Range.clip(right2,(float) -0.3,(float) 0.3);
        left2 = Range.clip(left2,(float) -0.3,(float) 0.3);
        rightone = Range.clip(rightone, var2, var);
        leftone = Range.clip(leftone, var2, var);
        relicpower = Range.clip(relicpower, (float) -0.4, (float) 0.4);
        
        // Servo (open)
         if(gamepad2.b){
            telemetry.addData("Servo", "B");
            servoRB.setPosition(1.45);
            servoRF.setPosition(1.45);
            servoLB.setPosition(1.45);
            servoLF.setPosition(1.45);
            //telemetry.addData("Position", liftermotor.getCurrentPosition());
            //telemetry.update();
         }
         // Servo (close)
         if(gamepad2.a){
            
            servoRB.setPosition(0.0077);
            servoRF.setPosition(0.25);
            servoLB.setPosition(0.01);
            servoLF.setPosition(0.4);
            telemetry.addData("Servo", "A");
            //telemetry.addData("Position", liftermotor.getCurrentPosition());
            //telemetry.update();
            //telemetry.addData("Position", servoLB.getPosition());
         }
        
        //Enables left to be initialized for motor power
        double left;
        left = -gamepad2.left_stick_y;
        //setting lifter power
        liftermotor.setPower(left);

        //setting driving motor powers
        drivefrontone.setPower(rightone);
        drivebackone.setPower(rightone);
        drivefronttwo.setPower(leftone);
        drivebacktwo.setPower(leftone);
        drivefrontone.setPower(right2);
        drivebackone.setPower(right2);
        drivefronttwo.setPower(left2);
        drivebacktwo.setPower(left2);
        
        relicthrower.setPower(relicpower);
        
        
    }
}
