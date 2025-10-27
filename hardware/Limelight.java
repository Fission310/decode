package org.firstinspires.ftc.teamcode.hardware;
import static org.firstinspires.ftc.teamcode.opmode.auton.LimelightConstants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;
import com.qualcomm.*;

import java.util.ArrayList;
import java.util.List;

public class Limelight extends Mechanism {
    private Limelight3A limelight;
    private ArrayList<Location> locations = new ArrayList<>();

    public class Location {
        public int tagID;
        public double x;     // left/right displacement (meters)
        public double y;     // forward/backward displacement (meters)
        public double yaw;   // yaw angle (degrees)
        public double score;

        public double distScore;
        public double rotScore;
        public double yScore;

        public Location(int tagID, double x, double y, double yaw) {
            this.tagID = tagID;
            this.x = x;
            this.y = y;
            this.yaw = yaw;
        }
    }

    public Limelight(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE);  // ensure pipeline configured for AprilTag
        limelight.start();
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            locations.clear();
            return;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        locations.clear();
        if (tags == null || tags.isEmpty()) return;

        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag == null) continue;

            int id = tag.getFiducialId();

            Pose3D pose = tag.getTargetPoseRobotSpace();
            Position data = pose.getPosition();
            YawPitchRollAngles angles = pose.getOrientation();
            if (pose == null || data == null) continue;

            // Access translation and rotation directly from the array
            double x_m = data.x;   // Left/right
            double y_m = data.y;   // Forward/back
            double z_m = data.z;   // Up/down
            double yawDeg = angles.getYaw(); // Yaw angle

            Location loc = new Location(id, x_m, y_m, yawDeg);
            locations.add(loc);
        }
    }




    public Location getBest() {
        if (locations.isEmpty()) {
            return new Location(-1, 0.0, 0.0, 0.0);
        }

        for (Location loc : locations) {
            loc.distScore = -X_WEIGHT * Math.abs(loc.x);
            loc.yScore    = -Y_WEIGHT * Math.abs(loc.y);
            loc.rotScore  = -ROT_WEIGHT * Math.abs(loc.yaw);
            loc.score     = loc.distScore + loc.yScore + loc.rotScore;
        }

        locations.sort((a, b) -> Double.compare(b.score, a.score));
        return locations.get(0);
    }

    public DriveCommands computeDriveCommands(double desiredFwdMeters, double forwardGain, double strafeGain, double turnGain) {
        Location best = getBest();
        double forward = (best.y - desiredFwdMeters) * forwardGain;
        double strafe  = best.x * strafeGain;
        double turn    = best.yaw * turnGain;
        return new DriveCommands(forward, strafe, turn);
    }

    public static class DriveCommands {
        public double forward;  // meters * gain
        public double strafe;   // meters * gain (positive = right)
        public double turn;     // degrees * gain (positive = clockwise)

        public DriveCommands(double forward, double strafe, double turn) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        Location best = getBest();
        telemetry.addData("Best Tag ID", best.tagID);
        telemetry.addData("Left/Right x (m)", String.format("%.3f", best.x));
        telemetry.addData("Forward y (m)", String.format("%.3f", best.y));
        telemetry.addData("Yaw (deg)", String.format("%.2f", best.yaw));
        telemetry.addData("Score", String.format("%.3f", best.score));
        telemetry.addData("Num Tags", locations.size());
        telemetry.update();
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }

    public void setPipeline(int pipeline) {
        if (limelight != null) limelight.pipelineSwitch(pipeline);
    }
}
