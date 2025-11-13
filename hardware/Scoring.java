package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.drive.Drivetrain;

/**
 * High-level scoring controller:
 * - Manual drive + intake/outtake
 * - One-button: auto-align to AprilTag, adjust velocity, spin up shooter, feed and shoot
 * - Helper method to start the same sequence from code (e.g., auton)
 */
@Config
public class Scoring extends Mechanism {

    // ------------------ SUBSYSTEMS ------------------
    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Transfer transfer;
    private final Shooter shooter;
    private final Limelight limelight;

    // ------------------ MODES / STATE ------------------
    private enum Mode {
        DRIVER,        // normal tele-op driving
        AUTO_ALIGN,    // using Limelight to center on tag
        SHOOTING       // shooter spinup + feeding
    }

    private enum ShootStage {
        NONE,
        SPINUP,
        FEEDING
    }

    private Mode mode = Mode.DRIVER;
    private ShootStage shootStage = ShootStage.NONE;

    // timing for shooting
    private double stageStartTime = 0.0;

    // ------------------ TUNABLE CONSTANTS (Dashboard) ------------------
    // Desired distance from tag when aligned (meters)
    public static double DESIRED_FWD_METERS = 1.5;

    // Gains for auto-align movement
    public static double FORWARD_GAIN = 1.0;
    public static double STRAFE_GAIN = 1.0;
    public static double TURN_GAIN = 0.03;

    // Max speed while auto-aligning (0..1)
    public static double MAX_AUTO_SPEED = 0.6;

    // Alignment tolerances
    public static double X_TOLERANCE = 0.03;          // side error (m)
    public static double Y_TOLERANCE = 0.05;          // forward error (m)
    public static double YAW_TOLERANCE_DEG = 2.0;     // angle error (deg)

    // Shooter timing
    public static double SHOOT_SPINUP_TIME = 0.5;     // seconds to spin up shooter
    public static double SHOOT_FEED_TIME = 1.0;       // seconds to feed balls

    // ------------------ BUTTON EDGE FLAGS ------------------
    private boolean shootButtonLatched = false;
    private boolean abortButtonLatched = false;

    // ------------------ CONSTRUCTOR ------------------
    public Scoring(LinearOpMode opMode) {
        this.opMode = opMode;

        drivetrain = new Drivetrain(opMode);
        intake = new Intake(opMode);
        transfer = new Transfer(opMode);
        shooter = new Shooter(opMode);
        limelight = new Limelight(opMode);
    }

    // ------------------ INIT ------------------
    @Override
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        intake.init(hwMap);
        transfer.init(hwMap);
        shooter.init(hwMap);
        limelight.init(hwMap);
    }

    // ------------------ TELEMETRY ------------------
    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Mode", mode);
        telemetry.addData("Shoot Stage", shootStage);

        // Show basic Limelight info (last best target)
        Limelight.Location loc = limelight.getBest();
        telemetry.addData("Tag ID", loc.tagID);
        telemetry.addData("LL x (m)", loc.x);
        telemetry.addData("LL y (m)", loc.y);
        telemetry.addData("LL yaw (deg)", loc.yaw);

        Pose2d pose = drivetrain.getPoseEstimate();
        telemetry.addData("Pose X", pose.getX());
        telemetry.addData("Pose Y", pose.getY());
        telemetry.addData("Pose H (rad)", pose.getHeading());
    }

    // ------------------ PUBLIC HELPERS ------------------

    /**
     * Request a full "center on tag, aim, and shoot" sequence.
     * Can be called from TeleOp (button) or auton code.
     */
    public void requestAutoAlignAndShoot() {
        if (mode == Mode.DRIVER) {
            mode = Mode.AUTO_ALIGN;
            shootStage = ShootStage.NONE;
        }
    }

    /**
     * Same idea as above; conceptually "go to center of shooting zone,
     * aim, and fire". The "center" is defined as DESIRED_FWD_METERS
     * away from the tag with small x/y/yaw error.
     */
    public void requestCenterShot() {
        requestAutoAlignAndShoot();
    }

    /**
     * Abort any auto-align / shooting and return to driver control.
     */
    private void abortAuto() {
        mode = Mode.DRIVER;
        shootStage = ShootStage.NONE;
        drivetrain.setDrivePower(new Pose2d(0, 0, 0));
        shooter.passivePower();
        transfer.downPos();
    }

    // ------------------ MAIN LOOP ------------------
    @Override
    public void loop(Gamepad gamepad) {
        // Update odometry & trajectory follower
        drivetrain.update();

        // Update Limelight measurements
        limelight.update();

        // Always handle intake / outtake
        handleIntake(gamepad);

        // Handle scoring button and abort button
        handleButtons(gamepad);

        // Run control logic based on current mode
        switch (mode) {
            case DRIVER:
                manualDrive(gamepad);
                break;

            case AUTO_ALIGN:
                autoAlignStep();
                break;

            case SHOOTING:
                autoShootStep();
                break;
        }
    }

    // ------------------ INPUT HANDLING ------------------

    private void handleButtons(Gamepad gamepad) {
        // SHOOT button: right stick button (see Controls.SHOOT)
        boolean shootPressed = gamepad.right_stick_button;
        if (shootPressed && !shootButtonLatched) {
            shootButtonLatched = true;
            requestAutoAlignAndShoot();
        } else if (!shootPressed) {
            shootButtonLatched = false;
        }

        // ABORT button: dpad_down (Controls.ABORT)
        boolean abortPressed = gamepad.dpad_down;
        if (abortPressed && !abortButtonLatched) {
            abortButtonLatched = true;
            abortAuto();
        } else if (!abortPressed) {
            abortButtonLatched = false;
        }
    }

    private void handleIntake(Gamepad gamepad) {
        // INTAKE: A
        // OUTTAKE: left bumper
        if (gamepad.a) {
            intake.intake();
        } else if (gamepad.left_bumper) {
            intake.outtake();
        } else {
            intake.stop();
        }
    }

    // ------------------ DRIVING ------------------

    private void manualDrive(Gamepad gamepad) {
        drivetrain.setDrivePower(new Pose2d(
                -gamepad.left_stick_y,   // forward/back
                -gamepad.left_stick_x,   // strafe
                -gamepad.right_stick_x   // rotate
        ));
    }

    private void autoAlignStep() {
        Limelight.Location loc = limelight.getBest();

        // If no tag seen, hold still and wait
        if (loc.tagID < 0) {
            drivetrain.setDrivePower(new Pose2d(0, 0, 0));
            return;
        }

        // Compute drive commands from Limelight helper (distance-aware gains)
        Limelight.DriveCommands cmds = limelight.computeDriveCommands(
                DESIRED_FWD_METERS, FORWARD_GAIN, STRAFE_GAIN, TURN_GAIN
        );

        double forward = clamp(cmds.forward, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double strafe  = clamp(cmds.strafe,  -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn    = clamp(cmds.turn,    -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

        drivetrain.setDrivePower(new Pose2d(forward, strafe, turn));

        // Check alignment tolerances
        double fwdError = loc.y - DESIRED_FWD_METERS;

        boolean aligned =
                Math.abs(loc.x)      < X_TOLERANCE &&
                        Math.abs(fwdError)   < Y_TOLERANCE &&
                        Math.abs(loc.yaw)    < YAW_TOLERANCE_DEG;

        if (aligned) {
            // Stop and begin shooting sequence
            drivetrain.setDrivePower(new Pose2d(0, 0, 0));
            beginShooting();
        }
    }

    private void beginShooting() {
        mode = Mode.SHOOTING;
        shootStage = ShootStage.SPINUP;
        stageStartTime = opMode.getRuntime();

        shooter.shoot();      // spin shooter up
        transfer.downPos();   // make sure transfer is down while spinning up
    }

    private void autoShootStep() {
        double now = opMode.getRuntime();
        drivetrain.setDrivePower(new Pose2d(0, 0, 0)); // stay still while shooting

        switch (shootStage) {
            case SPINUP:
                if (now - stageStartTime >= SHOOT_SPINUP_TIME) {
                    // Start feeding balls
                    transfer.upPos();
                    shootStage = ShootStage.FEEDING;
                    stageStartTime = now;
                }
                break;

            case FEEDING:
                if (now - stageStartTime >= SHOOT_FEED_TIME) {
                    // Done shooting, reset
                    transfer.downPos();
                    shooter.passivePower();
                    shootStage = ShootStage.NONE;
                    mode = Mode.DRIVER;
                }
                break;

            case NONE:
            default:
                // Safety fallback
                mode = Mode.DRIVER;
                break;
        }
    }

    // ------------------ UTIL ------------------
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
