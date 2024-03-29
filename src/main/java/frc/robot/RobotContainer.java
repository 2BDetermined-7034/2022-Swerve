
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.Auto.AutoFactory;
import frc.robot.commands.Drive.DefaultDriveCommand;

import frc.robot.commands.VIsion.AutoLock;
import frc.robot.subsystems.*;


public class RobotContainer {
  ShuffleboardTab tab = Shuffleboard.getTab("Match");
  private final SwerveDrive m_drivetrainSubsystem = new SwerveDrive();

  private final Vision m_vision = new Vision();
  SendableChooser<Command> m_autoSelector;

  //VisionCommand visionCommand = new VisionCommand(m_vision, m_drivetrainSubsystem);
  AutoLock m_autoLock = new AutoLock(m_vision, m_drivetrainSubsystem);

  // Controllers
  private final XboxController m_controller = new XboxController(0);


  public RobotContainer() {
    m_autoSelector = new SendableChooser<>();
    m_autoSelector.setDefaultOption("test auto", AutoFactory.getAuto(m_drivetrainSubsystem));
    m_autoSelector.addOption("spin auto", AutoFactory.getSpinAuto(m_drivetrainSubsystem));
    m_autoSelector.addOption("square auto", AutoFactory.getSquareAuto(m_drivetrainSubsystem));
    m_autoSelector.addOption("Small square auto", AutoFactory.getSmallSquare(m_drivetrainSubsystem));
    m_autoSelector.addOption("Small square spin auto", AutoFactory.getSmallSquareSpin(m_drivetrainSubsystem));

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -square(modifyAxis(m_controller.getLeftY()) * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND),
            () -> -square(modifyAxis(m_controller.getLeftX()) * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND),
            () -> -square(modifyAxis(m_controller.getRightX()) * SwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
    ));

    //m_vision.setDefaultCommand(visionCommand);


    tab.add("Auto", m_autoSelector);
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new Button(m_controller::getBackButton).whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    new Button(m_controller::getBButton).whenHeld(m_autoLock);

  }

  public Command getAutonomousCommand() {
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

  private static double square(double value) {
    return Math.copySign(value * value, value);
  }
}
