// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

    public static PhotonCamera camera = new PhotonCamera(Constants.Camera.camName);

    public static final Transform2d camToRobot = new Transform2d();

    /** Creates a new Vision. */
    public Vision() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Method to update a swerve drives position with a fiducial reading
     * @param swerveDrive drive with pose estimator
     */
    public void updateSwervePos(SwerveDrive swerveDrive) {
        PhotonPipelineResult res = camera.getLatestResult();
        Pose2d camPose = getPoseFromBest();
        if(camPose != null) {
            swerveDrive.addVisionMeasurement(camPose.transformBy(camToRobot), getTimestamp(res)); //transform compose with camToRobot pose2d
        }
    }

    /**
     * Gets the position of the Camera from the best identified fiducial
     * @return pose3d
     */
    public Pose2d getPoseFromBest(){
       PhotonPipelineResult res = camera.getLatestResult(); // Retrieves data from camera
        int targetID = getTargetID(res);
        SmartDashboard.putNumber("ID", targetID);
        if (targetID != -1) {
            PhotonTrackedTarget measuredTarget = res.getBestTarget();
            Pose3d targetPose = getFiducialPose(targetID);

            /*
              cameraHeightMeters – The physical height of the camera off the floor in meters.
              targetHeightMeters – The physical height of the target off the floor in meters. This should be the height of whatever is being targeted (i.e. if the targeting region is set to top, this should be the height of the top of the target).
              cameraPitchRadians – The pitch of the camera from the horizontal plane in radians. Positive values up.
              targetPitchRadians – The pitch of the target in the camera's lens in radians. Positive values up.
              targetYaw – The observed yaw of the target. Note that this *must* be CCW-positive, and Photon returns CW-positive.
              gyroAngle – The current robot gyro angle, likely from odometry.
              fieldToTarget – A Pose2d representing the target position in the field coordinate system.
              cameraToRobot – The position of the robot relative to the camera. If the camera was mounted 3 inches behind the "origin" (usually physical center) of the robot, this would be Transform2d(3 inches, 0 inches, 0 degrees).
             */

            return PhotonUtils.estimateFieldToRobot( //I'll comment later ChatGPT wrote this not my fault
                    Constants.Camera.camHeightOffGround, targetPose.getZ(),
                    Constants.Camera.cameraPitch, measuredTarget.getPitch(),
                    Rotation2d.fromDegrees(-measuredTarget.getYaw()),
                    SwerveDrive.getGyroscopeRotation(), targetPose.toPose2d(), camToRobot
                    );

        }
       return null;
    }

    /**
     * Get best targetID from the camera
     * @param res camera post processed results
     * @return ID of best fiducial
     */
    public int getTargetID(PhotonPipelineResult res) {
        if(res.hasTargets()) { // Dont remove you will get a nullptr-exception
            return res.getBestTarget().getFiducialId();
        }
        return -1;
    }

    /**
     * Gets timestamp of measurement
     * @param res camera post processed results
     * @return timestamp adjusted for latency
     */
    public double getTimestamp(PhotonPipelineResult res) {
        return Timer.getFPGATimestamp() - res.getLatencyMillis();
    }
    public Pose3d getFiducialPose(int fiducialID) {
        return Constants.Field.targetMap.get(fiducialID);
    }


}
