
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
public class Transfer extends Mechanism {
        private PIDController controller;

        private final DcMotorEx[] motor = new DcMotorEx[1];
        private Servo servo;

        private VoltageSensor voltage;

        public double down = 0.89; // 0.18
        public double up = 0.18;
        public double m = 1;

        public Transfer(LinearOpMode opMode) { this.opMode = opMode; }

        @Override
        public void init(HardwareMap hwMap) {
            voltage = hwMap.voltageSensor.iterator().next();
            motor[0] = hwMap.get(DcMotorEx.class, "transferMotor");
            servo = hwMap.get(Servo.class, "transferServo");
            servo.setPosition(down);

            motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor[0].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // might be wrong RunMode

            motor[0].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            motor[0].setDirection(DcMotorEx.Direction.REVERSE);

        }

        public void run() {
            motor[0].setPower(m);
        }

        public void stop() {
            motor[0].setPower(0);
        }

        public void upPos(){
            servo.setPosition(up);
            run();
        }
        public void downPos(){
            servo.setPosition(down);
            stop();
        }


        @Override
        public void loop(Gamepad gamepad) {
            if (gamepad.dpad_up) {
                upPos();
            } else{
                downPos();
            }
        }

    }
