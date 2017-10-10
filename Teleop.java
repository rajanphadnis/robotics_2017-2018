package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp



//@Disabled
public class Teleop extends OpMode {
    Servo servoRB;
    Servo servoLB;
    Servo servoRF;
    Servo servoLF;
    DcMotor liftermotor;
    DcMotor drivefrontone;
    DcMotor drivefronttwo;
    DcMotor drivebackone;
    DcMotor drivebacktwo;
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
        
       // liftermotor = hardwareMap.dcMotor.get("liftermotor");
        drivefrontone = hardwareMap.dcMotor.get("d1");
        drivefronttwo = hardwareMap.dcMotor.get("d2");
        drivebackone = hardwareMap.dcMotor.get("d3");
        drivebacktwo = hardwareMap.dcMotor.get("d4");


    }


    @Override
    public void loop() {
        // 4 u rug. gkys
        //if(gamepad2.a) {
            // long time = System.nanoTime();
            //liftermotor.setPower(0.3);
            
        //}
        float rightone = gamepad1.right_stick_y;
        float leftone = -gamepad1.left_stick_y;
        float var = (float) 0.5;
        float var2 = (float) -0.5;
        
        // clip the right/left values so that the values never exceed +/- 1

        rightone = Range.clip(rightone, var2, var);
        leftone = Range.clip(leftone, var2, var);
        
        
        if(gamepad1.b){
            
            servoRB.setPosition(0.90);
            servoRF.setPosition(0.90);
            servoLB.setPosition(0.90);
            servoLF.setPosition(0.90);
            
        }
        // Servo (close)
        if(gamepad1.a){
            servoRB.setPosition(0.2);
            servoRF.setPosition(0.2);
            servoLB.setPosition(0.25);
            servoLF.setPosition(0.25);
         
        }
        
        double left;
        left = -gamepad2.left_stick_y;

        drivefrontone.setPower(rightone);
        drivebackone.setPower(rightone);
        drivefronttwo.setPower(leftone);
        drivebacktwo.setPower(leftone);
        

        /*liftermotor.setPower(left);
        
        if(gamepad1.x)
        {
            liftermotor.setPower(0.50);
            
        }

        if(gamepad1.y)
        {
            liftermotor.setPower(-0.50);
        }*/
        
    }
}
