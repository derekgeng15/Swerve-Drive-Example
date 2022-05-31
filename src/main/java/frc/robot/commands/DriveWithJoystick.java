// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class DriveWithJoystick extends CommandBase {
  /** Creates a new DriveWithJoystick. */
  private final Chassis m_chassis;
  private final XboxController m_XboxController;

  public DriveWithJoystick(Chassis chassis, XboxController XboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_chassis = chassis;
    m_XboxController = XboxController;
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_chassis.set(ChassisSpeeds.fromFieldRelativeSpeeds(
        m_XboxController.getLeftX() * Constants.kChassis.kMaxMetersPerSecond,
        m_XboxController.getLeftY() *  Constants.kChassis.kMaxMetersPerSecond,
        m_XboxController.getRightX() * Constants.kChassis.kMaxRadiansPerSecond,
        m_chassis.getAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
