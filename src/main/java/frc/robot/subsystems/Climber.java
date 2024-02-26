// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants;
//import frc.robot.Constants.ManipulatorConstants;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final VictorSPX m_climb;

  public Climber() {
    //Constructs the Climber
    m_climb = new VictorSPX(Constants.ManipulatorConstants.kClimb_CANID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public void goUp(){
  //Allows the robot to go up
  m_climb.set(VictorSPXControlMode.PercentOutput, -.5);
}

public void stop(){
  //Stops the robot from going up
  m_climb.set(VictorSPXControlMode.PercentOutput, 0);
}

}
