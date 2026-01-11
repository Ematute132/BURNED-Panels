package org.firstinspires.ftc.teamcode.ILT.Java.LimeLightJava;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.DriveTrainJava;
import org.firstinspires.ftc.teamcode.next.subsystems.data.Motif;

import java.util.List;

public class LimeLightJava implements Subsystem {

    // Holds a reference to our Limelight3A camera.
    // Initialized in initialize().
    private static Limelight3A ll;

    // Basic Limelight targeting values.
    // TX = horizontal angle, TY = vertical angle, TA = target area percent on screen.
    private static double currentTx = 0.0;
    private static double currentTy = 0.0;
    private static double currentTa = 0.0;
    private static boolean hasValidTarget = false;

    // Which "motif" (tag encoding) is seen.
    private static Motif detectedMotif = Motif.NONE;

    // Fiducial (AprilTag) detection info.
    private static int fiducialCount = 0;
    private static String fiducialData = "No fiducials";

    @Override
    public void initialize() {
        // Grab the Limelight from the hardware map.
        ll = ActiveOpMode.hardwareMap().get(Limelight3A.class, "ll");

        // Set how often the Limelight updates data (100 Hz).
        ll.setPollRateHz(100);

        // Switch to pipeline 0.
        ll.pipelineSwitch(0);

        // Start the Limelight camera feed.
        ll.start();
    }

    @Override
    public void periodic() {
        // Update all Limelight-derived data every loop.
        updateBasicData();
        updateFiducialData();
        updateMotif();
    }

    // Grab basic tracking information like tx, ty, and ta.
    private static void updateBasicData() {
        if (ll == null) return;

        LLResult result = ll.getLatestResult();
        if (result != null && result.isValid()) {
            hasValidTarget = true;
            currentTx = result.getTx();
            currentTy = result.getTy();
            currentTa = result.getTa();
        } else {
            hasValidTarget = false;
            currentTx = 0.0;
            currentTy = 0.0;
            currentTa = 0.0;
        }
    }

    // Look for fiducials (AprilTags) and build a telemetry string.
    private static void updateFiducialData() {
        if (ll == null) return;

        LLResult result = ll.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            fiducialCount = fiducials.size();

            if (!fiducials.isEmpty()) {
                StringBuilder sb = new StringBuilder();
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    sb.append("ID: ").append(fr.getFiducialId()).append(", ");
                    sb.append("X: ").append(String.format("%.2f", fr.getTargetXDegrees())).append("°, ");
                    sb.append("Strafe: ")
                            .append(String.format("%.2f", fr.getRobotPoseTargetSpace().getPosition().getClass()))
                            .append("\n");
                }

                fiducialData = sb.toString().trim();
            } else {
                fiducialData = "No fiducials detected";
            }
        } else {
            fiducialCount = 0;
            fiducialData = "No valid result";
        }
    }

    // Map tag IDs to Motif values.
    private static void updateMotif() {
        if (ll == null) return;

        LLResult result = ll.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fR = result.getFiducialResults();
            if (!fR.isEmpty()) {
                LLResultTypes.FiducialResult f = fR.get(0);
                int id = f.getFiducialId();
                if (id == 21) {
                    detectedMotif = Motif.GPP;
                } else if (id == 22) {
                    detectedMotif = Motif.PGP;
                } else {
                    detectedMotif = Motif.PPG;
                }
            } else {
                detectedMotif = Motif.NONE;
            }
        } else {
            detectedMotif = Motif.NONE;
        }
    }

    // Raw access to LLResult if another subsystem needs full data.
    public static LLResult grabResultData() {
        if (ll == null) return null;

        LLResult lR = ll.getLatestResult();
        if (lR != null && lR.isValid()) {
            return lR;
        }
        return null;
    }

    // Use MegaTag pose to estimate robot position from tags.
    public static Pose megaTag() {
        if (ll == null) return null;

        LLResult lR = ll.getLatestResult();
        double yaw = DriveTrainJava.currentHeading;

        // Tell Limelight current robot yaw (usually in degrees).
        ll.updateRobotOrientation(yaw);

        LLResult result =  ll.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2();
                // Use botpose data
            }
        }

        return null;
    }

    // Build a pretty telemetry string summarizing status.
    public static String getTelemetryString() {
        StringBuilder sb = new StringBuilder();
        sb.append("=== LIMELIGHT STATUS ===\n");
        sb.append("Valid Target: ").append(hasValidTarget).append("\n");
        if (hasValidTarget) {
            sb.append("TX: ").append(String.format("%.2f", currentTx)).append("°\n");
            sb.append("TY: ").append(String.format("%.2f", currentTy)).append("°\n");
            sb.append("TA: ").append(String.format("%.2f", currentTa)).append("%\n");
        }
        sb.append("Fiducials: ").append(fiducialCount).append("\n");
        if (fiducialCount > 0) {
            sb.append(fiducialData).append("\n");
        }
        sb.append("Motif: ").append(detectedMotif);
        return sb.toString();
    }

    // Getters for external access to data
    public static double getCurrentTx() {
        return currentTx;
    }

    public static double getCurrentTy() {
        return currentTy;
    }

    public static double getCurrentTa() {
        return currentTa;
    }

    public static boolean hasValidTarget() {
        return hasValidTarget;
    }

    public static Motif getDetectedMotif() {
        return detectedMotif;
    }

    public static int getFiducialCount() {
        return fiducialCount;
    }

    public static String getFiducialData() {
        return fiducialData;
    }
}