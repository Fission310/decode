package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.stuyfission.fissionlib.util.Mechanism;


import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
public class Shooter extends Mechanism {
    private PIDController controller;

    private final DcMotorEx[] motors = new DcMotorEx[2];
    private Servo servo;

    private VoltageSensor voltage;

    public static double shootPwr = 0.89; // 0.18
    public static double passPwr = 0.18;
    public static double srvo = 0;

    public Shooter(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        voltage = hwMap.voltageSensor.iterator().next();
        servo = hwMap.get(Servo.class, "shooterServo");

        servo.setPosition(srvo);
        motors[0] = hwMap.get(DcMotorEx.class, "shooterLeftMotor");
        motors[1] = hwMap.get(DcMotorEx.class, "shooterRightMotor");

        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motors[0].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // might be wrong RunMode
        motors[1].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motors[0].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motors[0].setDirection(DcMotorEx.Direction.REVERSE);
        motors[1].setDirection(DcMotorEx.Direction.REVERSE);

    }

    public void shoot() {
        motors[0].setPower(shootPwr);
        motors[1].setPower(shootPwr);
    }

    public void passivePower() {
        motors[0].setPower(passPwr);
        motors[1].setPower(passPwr);
    }

    public void setPower(double d){
        shootPwr = d;
    }

    public void adjustPower(){
//gonna be the auto power adjust but limelight needs to be done (and tested) first
    }
    public void setServo(double d){
        servo.setPosition(d);
    }

    public void defaultServo(){
        srvo = 0;
        setServo(srvo);
    }



    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            shoot();
        } else{
            passivePower();
        }
    }

}
