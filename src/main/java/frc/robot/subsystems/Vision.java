// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Transform2d;

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

    public static void update(SwerveDrive swerveDrive) {
        var res = camera.getLatestResult(); // Retrieves data from camera
        if(res.hasTargets()) { // Dont remove you will get a nullptr-exception

            double imageCaptureTime = Timer.getFPGATimestamp() - res.getLatencyMillis(); // Vision Pose Measurement Timestamp

            Transform3d camToTargetTrans = res.getBestTarget().getBestCameraToTarget();

            int targetID = res.getBestTarget().getFiducialId();

            Pose3d camPose = Constants.Field.targetMap.get(targetID).transformBy(camToTargetTrans.inverse()); //Inverts the transform between target and camera
            swerveDrive.addVisionMeasurement(camPose.toPose2d().transformBy(camToRobot), imageCaptureTime); //transform campose with camToRobot pose2d
        }
    }

    public static boolean getHasTargets() {
        return camera.getLatestResult().hasTargets();

    }

}
