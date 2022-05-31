// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. */
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private SwerveModule[] m_modules;
  private SwerveModuleState[] m_states;
  private final AHRS m_navX;

  private final SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry m_pose;
  private final Field2d m_field;
  


  private ChassisSpeeds m_driveSignal;
  private Rotation2d m_angle;
  public Chassis() {
    m_frontLeft = Mk4iSwerveModuleHelper.createFalcon500(Constants.kChassis.kGearRatio,
        Constants.kCANID.kFrontLeftDrive, Constants.kCANID.kFrontLeftSteer, Constants.kCANID.kFrontLeftCANCoder, 0);
    m_frontRight = Mk4iSwerveModuleHelper.createFalcon500(Constants.kChassis.kGearRatio,
        Constants.kCANID.kFrontRightDrive, Constants.kCANID.kFrontRightSteer, Constants.kCANID.kFrontRightCANCoder, 0);
    m_backLeft = Mk4iSwerveModuleHelper.createFalcon500(Constants.kChassis.kGearRatio, Constants.kCANID.kBackLeftDrive,
        Constants.kCANID.kBackLeftSteer, Constants.kCANID.kBackLeftCANCoder, 0);
    m_backRight = Mk4iSwerveModuleHelper.createFalcon500(Constants.kChassis.kGearRatio,
        Constants.kCANID.kBackRightDrive, Constants.kCANID.kBackRightSteer, Constants.kCANID.kBackRightCANCoder, 0);

    m_navX = new AHRS(Port.kMXP);

    m_modules[0] = m_frontLeft;
    m_modules[1] = m_frontRight;
    m_modules[2] = m_backLeft;
    m_modules[3] = m_backRight;

    m_kinematics = new SwerveDriveKinematics(
        new Translation2d(Constants.kChassis.kTrackWidth / 2, Constants.kChassis.kTrackWidth / 2),
        new Translation2d(Constants.kChassis.kTrackWidth / 2, -Constants.kChassis.kTrackWidth / 2),
        new Translation2d(-Constants.kChassis.kTrackWidth / 2, Constants.kChassis.kTrackWidth / 2),
        new Translation2d(-Constants.kChassis.kTrackWidth / 2, -Constants.kChassis.kTrackWidth / 2));

    m_angle = new Rotation2d();
    m_pose = new SwerveDriveOdometry(m_kinematics, m_angle);
    m_driveSignal = new ChassisSpeeds();
    m_states = new SwerveModuleState[4];
    

    m_field = new Field2d();
  }

  public void set(ChassisSpeeds chassisSpeeds){
    SwerveModuleState[] dstates = m_kinematics.toSwerveModuleStates(m_driveSignal);
    SwerveDriveKinematics.desaturateWheelSpeeds(dstates, Constants.kChassis.kMaxMetersPerSecond);
    for(int i = 0; i < 4; i++)
      m_modules[i].set(dstates[i].speedMetersPerSecond / Constants.kChassis.kMaxMetersPerSecond, dstates[i].angle.getRadians());
    m_driveSignal = chassisSpeeds;
  }

  public Rotation2d getAngle(){
    return m_navX.getRotation2d();
  }

  public void setPose(Pose2d pose){
    m_pose.resetPosition(pose, m_navX.getRotation2d());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for(int i = 0; i < 4; i++){
      m_states[i].speedMetersPerSecond = m_modules[i].getDriveVelocity();
      m_states[i].angle = new Rotation2d(m_modules[i].getSteerAngle());
    }
    m_pose.update(m_angle, m_states);
    m_field.setRobotPose(m_pose.getPoseMeters());

    SmartDashboard.putNumber("X", m_pose.getPoseMeters().getX());
    SmartDashboard.putNumber("Y", m_pose.getPoseMeters().getY());
    SmartDashboard.putNumber("Theta", m_pose.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putData("Field", m_field);
  }
}
