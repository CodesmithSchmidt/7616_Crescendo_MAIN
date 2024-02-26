// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;

public class Arm2 extends SubsystemBase {
  /** Creates a new Arm2. */
 public static final int
  armPIDSlot = 0;
 
  private static final double
    armNormal_P = .025,
    armNormal_I = 0.0,
    armNormal_D = 0.0;

  private final double
    nominalOutForward = 0.01,
    peakOutForward = 1.0,
    nominalOutReverse = -0.01,
    peakOutReverse = -0.6;

  private final double thresholdDEG = 2.0;

  private CANSparkMax arm1;
  private CANSparkMax arm2;
  private SparkPIDController armPIDController;
  private AbsoluteEncoder armAbsoluteEncoder;
 
  public Arm2() {
    arm1 = new CANSparkMax(Constants.ManipulatorConstants.kArm1_CANID, MotorType.kBrushless);
    arm2 = new CANSparkMax(Constants.ManipulatorConstants.kArm2_CANID, MotorType.kBrushless);
    arm1.restoreFactoryDefaults();
    arm2.restoreFactoryDefaults();
    arm2.follow(arm1, true);
    
    armPIDController = arm1.getPIDController();
    armAbsoluteEncoder = arm1.getAbsoluteEncoder(Type.kDutyCycle);
    armAbsoluteEncoder.setInverted(true);
    armAbsoluteEncoder.setPositionConversionFactor(360.0);
    armPIDController.setFeedbackDevice(armAbsoluteEncoder);

    armPIDController.setPositionPIDWrappingEnabled(false);

    armPIDController.setP(armNormal_P);
    armPIDController.setI(armNormal_I);
    armPIDController.setD(armNormal_D);
  }

   public void resetEncoders() {
    armPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  public void setSpeed(Supplier<Double> speedSupplier) {
    arm1.set((speedSupplier.get() * 0.5));
  }

  public void setPosition(double pos) {
    armPIDController.setReference(pos, CANSparkMax.ControlType.kPosition);
  }
  //(probably should) give this method the "setpoint", and subtract the current position
  public double getError() {
    return 0.0;
  }
//Should not return true until Error is less then set threshold.
  public boolean isFinishedMoving() {
    return getError() < thresholdDEG;
  }
//Checks if its at position
  public boolean isAtPosition(ArmPosition pos) {
    return Math.abs(getArmPosition() - pos.degreePos) < thresholdDEG;
  }
//Gets arm position
  public double getArmPosition() {
    return armAbsoluteEncoder.getPosition();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm encoder", getArmPosition());
    SmartDashboard.putNumber("arm error", getError());
    SmartDashboard.putBoolean("arm is finished", isFinishedMoving());



  }

//Goes to positions based on what it is going to look at.
  public enum ArmPosition {
    FLOOR(0.0),
    AMP(90.0),
    SPEAKER(15.0),
    HOOKSET(90.0),
    HOOKRELEASE(75.0);

  public final double degreePos;
      ArmPosition(double degreePos) {
        this.degreePos = degreePos;
      }
    }

}
