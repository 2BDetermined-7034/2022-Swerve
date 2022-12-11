// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Transform2d;
import org.photonvision.targeting.PhotonPipelineResult;

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
        Pose3d camPose = getPoseFromBest();
        swerveDrive.addVisionMeasurement(camPose.toPose2d().transformBy(camToRobot), getTimestamp(res)); //transform compose with camToRobot pose2d
    }

    /**
     * Gets the position of the Camera from the best identified fiducial
     * @return pose3d
     */
    public Pose3d getPoseFromBest(){
       PhotonPipelineResult res = camera.getLatestResult(); // Retrieves data from camera
        int targetID = getTargetID(res);
        SmartDashboard.putNumber("ID", targetID);
        if (targetID != -1) {
            Transform3d camToTargetTrans = res.getBestTarget().getBestCameraToTarget();

            SmartDashboard.putNumber("transform x", camToTargetTrans.getX());
            SmartDashboard.putNumber("transform y", camToTargetTrans.getY());
            SmartDashboard.putNumber("transform t", camToTargetTrans.getRotation().toRotation2d().getDegrees());

            //TODO: Fixed by inverting transform
            return getFiducialPose(targetID).transformBy(camToTargetTrans.inverse()); //Inverts the transform between FiducialPose and Camera to get CameraPose
        }
       return null;
    }

    /**
     * Get best targetID from the camera
     * @param res camera post processed results
     * @return ID of best fiducial
     */
    public int getTargetID(PhotonPipelineResult res) {
        if (res.hasTargets()) { // Dont remove you will get a nullptr-exception
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
