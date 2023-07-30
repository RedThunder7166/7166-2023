// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  private Spark m_leftBlinkin = new Spark(Constants.LED.LEFT_BLINKIN_ID);
  // private Spark m_rightBlinkin = new Spark(Constants.LED.RIGHT_BLINKIN_ID);

  public InstantCommand defaultColor;
  public InstantCommand rainbow;
  public InstantCommand solidYellow;
  public InstantCommand solidPurple;
  public InstantCommand strobeWhite;

  
  ShuffleboardTab tab = Shuffleboard.getTab("LED");
  // GenericEntry lednumberEntry = tab.add("LED #", 1).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
  

  public LEDSubsystem() {
    defaultColor = new InstantCommand(() -> setBlinkins(0.45), this);
    rainbow = new InstantCommand(() -> setBlinkins(-0.89), this);
    solidYellow = new InstantCommand(() -> setBlinkins(0.69), this);
    solidPurple = new InstantCommand(() -> setBlinkins(0.91), this);
    strobeWhite = new InstantCommand(() -> setBlinkins(-0.05), this); 
  
  }

  @Override
  public void periodic() {
    
    // setBlinkins(lednumberEntry.getDouble(1));

  }


  public void setLeftBlinkin(double speed) {
    m_leftBlinkin.set(speed);
  }
  public void setRightBlinkin(double speed) {
    // m_rightBlinkin.set(speed);
  }
  public void setBlinkins(double speed) {
    setLeftBlinkin(speed);
    // setRightBlinkin(speed);
  }
}
