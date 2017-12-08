package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.TouchSensor;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import android.graphics.Color;
@TeleOp



//@Disabled
public class Teleop2Persons extends OpMode {
    
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
    //TouchSensor touchsensor;
    
    //private ElapsedTime runtime = new ElapsedTime();
    
    Servo jt;
   long setTime = System.currentTimeMillis();
   boolean hasRun = false;
   // Assignments for lift, however UNUSED CODE
    final double LIFTERMOTORUP      = 1;                            // sets rate to move servo
    final double LIFTERMOTORDOWN      = -1;
    
    
    
    
    @Override
    public void init() {


        
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
        jt = hardwareMap.get(Servo.class, "jt");
        
        
        


    }
    @Override
    public void loop() {
        //float hsvValues[] = {0F,0F,0F};
        //final float values[] = hsvValues;
        /*float rightone = gamepad1.left_stick_y;
        float leftone = -gamepad1.left_stick_y;
        float speed = gamepad1.right_stick_x;
        float right2 = -gamepad1.right_trigger;
        float left2 = gamepad1.left_trigger;*/
        float rightone = gamepad1.right_stick_y;
        float leftone = -gamepad1.left_stick_y;
        float relicpower = gamepad2.right_stick_y;
        
        float var = (float) 0.75;
        float var2 = (float) -0.75;
        
         //clip the right/left values so that the values never exceed +/- 1
         
        // Creating motor power ranges for driving and relic thrower 
        rightone = Range.clip(rightone,(float) -1,(float) 1);
        leftone = Range.clip(leftone,(float) -1,(float) 1);
        //rightone = Range.clip(rightone, var2, var);
        //leftone = Range.clip(leftone, var2, var);
        //speed = Range.clip(speed, (float) -1.0, (float) 1.0);
        relicpower = Range.clip(relicpower, (float) -1, (float) 1);
        
        telemetry.update();
        // Servo (open)
         if(gamepad2.b){
            servoRB.setPosition(1.45);
            servoRF.setPosition(1.45);
            servoLB.setPosition(1.45);
            servoLF.setPosition(1.45);
         }
         // Servo (close)
         if(gamepad2.a){
            
            servoRB.setPosition(0.0077);
            servoRF.setPosition(0.25);
            servoLB.setPosition(0.01);
            servoLF.setPosition(0.4);
            telemetry.addData("Servo", "A");
         }
         if(gamepad2.y) {
             jt.setPosition(0);
         }
         if(gamepad2.x) {
             jt.setPosition(0.75);
         }
        if(gamepad1.y) {
            
            telemetry.addData("Moving", "Junk");
            servoRB.setPosition(1.45);
            servoRF.setPosition(1.45);
            servoLB.setPosition(1.45);
            servoLF.setPosition(1.45);
            telemetry.addData("Moved", "Junk");

                // Time stuff
            long t= System.currentTimeMillis();
            long end = t+1500;
            while(System.currentTimeMillis() < end) {
                // Moving motor
                liftermotor.setPower(1.0);
                
                // Thread.sleep(1500);
                
                
            }
            liftermotor.setPower(0.0);
        
            
        }
        if(gamepad1.x) {
            // Pick up and Move
            telemetry.addData("Moving", "Junk");
            servoRB.setPosition(0.0077);
            servoRF.setPosition(0.25);
            servoLB.setPosition(0.01);
            servoLF.setPosition(0.4);
            telemetry.addData("Moved", "Junk");
            // Time stuff
            long t= System.currentTimeMillis();
            long end = t+1500;
            while(System.currentTimeMillis() < end) {
                // Moving motor
                liftermotor.setPower(-0.5);
                
                // Thread.sleep(1500);
                
                
            }
            liftermotor.setPower(0.0);
            
        }
        
        //Enables left to be initialized for motor power
        double left = -gamepad2.left_stick_y;
        //setting lifter power
        liftermotor.setPower(left);
        
        
        

        //setting driving motor powers
        //drivefrontone.setPower(rightone+speed);
        //drivebackone.setPower(rightone+speed);
        //drivefronttwo.setPower(leftone+speed);
        //drivebacktwo.setPower(leftone+speed);
        drivefrontone.setPower(rightone);
        drivebackone.setPower(rightone);
        drivefronttwo.setPower(leftone);
        drivebacktwo.setPower(leftone);
        
        relicthrower.setPower(relicpower);
        
        
    }
}
