// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.RobotConstants;;


public class Scoring extends SubsystemBase {

  private final TalonFX 
m_shooter1 = new TalonFX(RobotConstants.kShooter1_CANID);
private final TalonFX
m_shooter2 = new TalonFX(RobotConstants.kShooter2_CANID);
private final CANSparkMax
m_intake = new CANSparkMax(RobotConstants.kIntake1_CANID, MotorType.kBrushless);

  /** Creates a new Scoring. */
  public Scoring() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
//Shooting
public void shooting(){
  m_shooter1.set(-0.60);
  m_shooter2.set(-0.60);
}
//Not shooting
public void stopShooting(){
  m_shooter1.set(0);
  m_shooter2.set(0);
}
//Pickup
public void intake(){
  m_intake.set(0.75);
}
//Stop pickup
public void intakeStop(){
  m_intake.set(0);
}
//Score amp/backward intake
public void intakeBack(){
  m_intake.set(-.6);
}

}
