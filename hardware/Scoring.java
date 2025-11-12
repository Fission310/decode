package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.drive.Drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.stuyfission.fissionlib.command.CommandSequence;
import com.stuyfission.fissionlib.input.GamepadStatic;

@Config
public class Scoring extends Mechanism {
    private Drivetrain drivetrain = new Drivetrain(opMode); // OPMODE NULL HERE
    private Intake intake = new Intake(opMode);
    private Transfer transfer = new Transfer(opMode);
    public Shooter shooter = new Shooter(opMode);

    public Pose2d topPos = new Pose2d(75,75,45);

    private State state = State.EMPTY;

    private enum State {
        FULL,
        SHOOTING,
        EMPTY,
    }

    public static double SHOT_WAIT = 0.1;

    private boolean climbPressed = false;
    private boolean frontClicked = false;
    private boolean dpadClicked = false;
    private boolean rightStickClicked = false;
    private boolean shortIntake = false;
    private boolean scoreClicked = false;

    public Scoring(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    private Command intakeCommand = () -> intake.intake();
    private Command outakeCommand = () -> intake.outtake();
    private Command stopIntake = () -> intake.stop();
    private Command shoot = () -> shooter.shoot();
    private Command resetShot = () -> shooter.passivePower();


    @Override
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        intake.init(hwMap);
        shooter.init(hwMap);
        transfer.init(hwMap);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("state", state);
        telemetry.addData("scoreClicked", scoreClicked);
        drivetrain.telemetry(telemetry);
        intake.telemetry(telemetry);
        shooter.telemetry(telemetry);
        transfer.telemetry(telemetry);
    }

    @Override
    public void loop(Gamepad gamepad){
        drivetrain.loop(gamepad);
        switch (state){
            case FULL:
                if (GamepadStatic.isButtonPressed(gamepad, Controls.SHOOT)){
                    state = State.SHOOTING;
                    shooter.shoot();
                   // state = State.SHOOTING;
                }
                else{
                    shooter.passivePower();
                }
            case EMPTY:

        }
    }
}
