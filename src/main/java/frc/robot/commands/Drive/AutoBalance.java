// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {

  
  private  Swerve swerve;

  private double currentAngle;
  private double drivePower;

  /** Creates a new AutoBalance. */
  public AutoBalance(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

    // Use addRequirements() here to declare subsystem dependencies.
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.zeroGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.currentAngle = swerve.getPitch();

    double kP = 0.03;
    drivePower = -0.4;
    //kP * currentAngle;
    // drivePower = Math.min(currentAngle, 0.0);
    drivePower = Math.copySign(drivePower, currentAngle);
    if(Math.abs(currentAngle) < 13){
      drivePower = 0;

    }
    // drivePower = copysign(drivePower, currentAngle);
      

    swerve.drive(new Translation2d(drivePower, 0), 0, true, true);
    
    // Debugging Print Statments
    System.out.println("Current Angle: " + currentAngle);
    // System.out.println("Error " + error);
    System.out.println("Drive Power: " + drivePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //swerve.drive(new Translation2d(0, 0), 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(error) < 1; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
    return false;
  }
}