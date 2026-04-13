package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoAim extends BaseHardware{ // naj added this to comply with standard baseHardware class

    public Limey limey;
    public Robot robot;
    public Turret turret;
    private DriveTrain driveTrain;

    // Distance behind the tag to aim at
    private static final double OFFSET_INCHES = 8.0;          // MJD
    private static final double OFFSET = OFFSET_INCHES * 0.0254; // meters // MJD

    @Override // naj added this to comply with standard baseHardware class
    public void init() {}
    public void init_loop() {}
    public void start() {}
    public void stop() {}
    public void loop() {}

   /* public AutoAim(Limey limey, Turret turret, DriveTrain driveTrain) {
        this.limey = limey;
        this.turret = turret;
        this.driveTrain = driveTrain;
    } */

    /**
     * CAMERA-SPACE AUTO AIM
     * Uses only what the camera sees (no field map) to aim
     * at a point OFFSET inches behind the AprilTag.
     *
     * Returns: turret-relative angle (degrees) to command.
     */
    public double computeAimAngle() {   // MJD

        if (robot == null || robot.limey == null){
            return Double.NaN;
        }

        // Must see april tag
        if (robot.limey.getTagID() == -1) return Double.NaN;   // MJD

        double[] tagCam = robot.limey.getTagPoseCameraSpace3D();   // MJD
        if (tagCam == null || tagCam.length < 3) return Double.NaN;   // MJD

        double tagX = tagCam[0];   // camera X (right +)
        double tagY = tagCam[1];   // camera Y (down +)
        double tagZ = tagCam[2];   // camera Z (forward +)

        double tagYawDeg = robot.limey.getTagYawCameraSpaceDeg();   // MJD
        if (Double.isNaN(tagYawDeg)) return Double.NaN;       // MJD

        double tagYawRad = Math.toRadians(tagYawDeg);         // MJD

        // In Limelight camera space, yaw is rotation about Y (up) axis.
        // Forward direction of the tag in camera frame:
        //   forward = (sin(yaw), 0, cos(yaw))  // MJD
        double ux = Math.sin(tagYawRad);   // X component of tag forward // MJD
        double uz = Math.cos(tagYawRad);   // Z component of tag forward // MJD

        double aimX = tagX - ux * OFFSET;   // MJD
        double aimZ = tagZ - uz * OFFSET;   // MJD

        double yawToAimDeg = Math.toDegrees(Math.atan2(aimX, aimZ));   // MJD

        double turretAngle = turret.getCurrentAngle();   // degrees, 0 = forward // MJD
        double turretTarget = yawToAimDeg - turretAngle;   // MJD

        turretTarget = ((turretTarget + 540) % 360) - 180;   // MJD

        Telemetry t = robot.limey.telemetry;
        if (t != null) {
            t.addData("AA TagCamX", tagX);
            t.addData("AA TagCamY", tagY);
            t.addData("AA TagCamZ", tagZ);
            t.addData("AA TagYawCamDeg", tagYawDeg);
            t.addData("AA AimX", aimX);
            t.addData("AA AimZ", aimZ);
            t.addData("AA YawToAimDeg", yawToAimDeg);
            t.addData("AA TurretAngle", turretAngle);
            t.addData("AA TurretTarget", turretTarget);
        }

        return turretTarget;   // MJD
    }
}
