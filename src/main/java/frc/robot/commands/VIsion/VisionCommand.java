package frc.robot.commands.VIsion;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class VisionCommand extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final Vision vision;

    public VisionCommand(Vision m_vision, SwerveDrive m_swerveDrive) {

        this.swerveDrive = m_swerveDrive;
        this.vision = m_vision;

        addRequirements(m_swerveDrive, m_vision);
    }

    @Override
    public void execute() {
        vision.updateSwervePos(swerveDrive);
    }

    @Override
    public void end(boolean interrupted) {

    }
}