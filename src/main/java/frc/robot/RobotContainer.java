
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.Auto.SimpleAuto;
import frc.robot.commands.Drive.DefaultDriveCommand;

import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  ShuffleboardTab tab = Shuffleboard.getTab("Match");
  private final SwerveDrive m_drivetrainSubsystem = new SwerveDrive();
  SendableChooser<Command> m_autoSelector;

  // Controllers
  private final XboxController m_controller = new XboxController(0);
  SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(Constants.DriveBase.xRateLimit);
  SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(Constants.DriveBase.yRateLimit);
  SlewRateLimiter m_rotspeedLimiter = new SlewRateLimiter(Constants.DriveBase.rotRateLimit);



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_autoSelector = new SendableChooser<>();
    m_autoSelector.addOption("test", SimpleAuto.getAuto(m_drivetrainSubsystem));



    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_yspeedLimiter.calculate(m_controller.getLeftY()) * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND),
            () -> -modifyAxis(m_xspeedLimiter.calculate(m_controller.getLeftX()) * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND),
            () -> -modifyAxis(m_rotspeedLimiter.calculate(m_controller.getRightX() / 2) * SwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
    ));



    tab.add("Auto", m_autoSelector);
    // Configure the button bindings
    configureButtonBindings();
  }


  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(m_controller::getBackButton).whenPressed(m_drivetrainSubsystem::zeroGyroscope);

  }


  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoSelector.getSelected();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.075);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }


}
