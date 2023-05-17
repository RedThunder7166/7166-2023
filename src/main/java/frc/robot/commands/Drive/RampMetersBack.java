// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class RampMetersBack extends CommandBase {
  /** Creates a new DriveMeters. */
  private Swerve s_Swerve;
  private double targetMetersX;
  private double targetMetersY;
  private double targetRotation;
  private Pose2d currentPose;
 private double feedForward; 

  private double x_error;
  private double y_error;
  private double rot_error;
  
  public RampMetersBack(Swerve swerve, double targetX, double targetY, double targetRot) {
    s_Swerve = swerve;
    targetMetersX = targetX;
    targetMetersY = targetY;
    targetRotation = targetRot;
    feedForward =-3.5;

    addRequirements(swerve);
  }

  public RampMetersBack(Swerve swerve, double targetX, double targetY, double targetRot, double feedForward) {
    s_Swerve = swerve;
    targetMetersX = targetX;
    targetMetersY = targetY;
    targetRotation = targetRot;
    this.feedForward = feedForward;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x_error = 0;
    y_error = 0;
    rot_error = 0;
    s_Swerve.resetOdometry(new Pose2d(0,0, s_Swerve.getYaw()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = s_Swerve.getPose();
    x_error = targetMetersX - currentPose.getX();
    y_error = targetMetersY - currentPose.getY();
    rot_error = targetRotation - currentPose.getRotation().getRadians();
    // System.out.println(x_error);
    // System.out.println(y_error);
    // System.out.println(rot_error);

    double x_kP = 4.3;
    double y_kP = 4.3;
    double rot_kP = 0.1;

    double x = x_kP * x_error + feedForward;
    double y = y_kP * y_error ;
    double rot = rot_kP * rot_error;
    s_Swerve.drive(
        new Translation2d(x, y),
        0, //rot,
        true,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(
        new Translation2d(0, 0),
        0,
        true,
        false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (Math.abs(x_error) <= 0.1 && Math.abs(y_error) <= 0.3 && Math.abs(rot_error) <= Math.toRadians(1)) {
    if (Math.abs(currentPose.getX()) >= Math.abs(targetMetersX) && Math.abs(currentPose.getY()) >= Math.abs(targetMetersY)) {
      return true;
    }
    return false;
  }
}
