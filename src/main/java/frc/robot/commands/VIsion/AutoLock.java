package frc.robot.commands.VIsion;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

import static frc.robot.Constants.Vision.*;

public class AutoLock extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final Vision vision;
    private final PIDController pidController;

    public AutoLock(Vision m_vision, SwerveDrive m_swerveDrive) {

        this.swerveDrive = m_swerveDrive;
        this.vision = m_vision;

        pidController = new PIDController(kPRotate, 0 ,0);
        pidController.enableContinuousInput(-180, 180);

        addRequirements(m_swerveDrive, m_vision);
    }

    @Override
    public void execute() {
        Pose3d tagPose = vision.getPoseFromBest();
        if (tagPose != null) {
            SmartDashboard.putNumber("april tag angle", Math.toDegrees(tagPose.getRotation().getAngle()));
            SmartDashboard.putNumber("april tag x", Math.toDegrees(tagPose.getX()));
            swerveDrive.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            0,
                            0,
                            pidController.calculate(0, 0),
                            swerveDrive.getGyroscopeRotation()
                    )
            );
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}