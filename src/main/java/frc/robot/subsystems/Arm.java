// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.RobotConstants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  //Declare variables
  private CANSparkMax m_motor, m_motor2;
  private SparkPIDController m_pidController;
  private AbsoluteEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;


  public Arm() {
    //Initialize objects
    m_motor = new CANSparkMax(RobotConstants.kArm1_CANID, MotorType.kBrushless);
    m_motor2 = new CANSparkMax(RobotConstants.kArm2_CANID, MotorType.kBrushless);
    
    m_motor.restoreFactoryDefaults();
    m_motor2.restoreFactoryDefaults();

    //set motor 2 to follow motor 1
    m_motor2.follow(m_motor, true);
   
    /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */

    m_pidController = m_motor.getPIDController();

    //Encoder Object created to display position values
    m_encoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);

    m_pidController.setFeedbackDevice(m_encoder);
  //  m_pidController2.setFeedbackDevice(m_encoder);

    // set PID coefficients **NEEDS TUNING**
    kP = 0;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    //Smart Motion Coefficients
    maxVel = 2000;
    maxAcc = 1500;
    allowedErr = .001;
        // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);


int smartMotionSlot = 0;
m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
   
    // display SmartMotion coefficients on SmartDashboard
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

    //Button to toggle between Velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double maxV = SmartDashboard.getNumber("Max Velocity", 0);
        double minV = SmartDashboard.getNumber("Min Velocity", 0);
        double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
        double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
    
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { m_pidController.setP(p); kP = p; }
        if((i != kI)) { m_pidController.setI(i); kI = i; }
        if((d != kD)) { m_pidController.setD(d); kD = d; }
        if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
          m_pidController.setOutputRange(min, max); 
          kMinOutput = min; kMaxOutput = max; 
        }

        if((maxV != maxVel)) {m_pidController.setSmartMotionMaxVelocity(maxV, 0); maxVel=maxV; }
        if((minV != minVel)) {m_pidController.setSmartMotionMinOutputVelocity(minV, 0); minVel=minV; }
        if((maxA != maxAcc)) {m_pidController.setSmartMotionMaxAccel(maxA, 0); maxAcc=maxA; }
        if((allE != allowedErr)) {m_pidController.setSmartMotionAllowedClosedLoopError(allE, 0); allowedErr=allE; }
 
        double setPoint, processVariable;
        processVariable = m_encoder.getPosition();
        setPoint = .333;
        /*
        boolean mode = SmartDashboard.getBoolean("Mode", false);
        if(mode){
          setPoint = SmartDashboard.getNumber("Set Velocity", 0);
          m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);

        } else {
          setPoint = SmartDashboard.getNumber("Set Position", 0);
          m_pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
          processVariable = m_encoder.getPosition();
        }
*/
        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("ProcessVariable", processVariable);
        SmartDashboard.putNumber("Output", m_motor.getAppliedOutput());

  }
  //This doesn't work when called from RobotContainer with a lambda expression command
  public void goToSetPoint(double setPoint){
    m_pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }


  //test functions just to make sure that the motors work **They do; these functions work
  public void doSomethingPlease(){
    m_motor.set(.5);
  }

    public void stopSomethingPlease(){
    m_motor.set(0);
  }

}
