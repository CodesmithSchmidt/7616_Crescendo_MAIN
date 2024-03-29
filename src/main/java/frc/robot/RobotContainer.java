// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
/*import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
*/
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
//import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Auto_1;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Arm2;
import frc.robot.subsystems.Scoring;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Arm2 m_robotArm = new Arm2();
  private final Scoring m_robotScorer = new Scoring();
  private final Climber m_robotClimber = new Climber();

  private final Auto_1 m_auto = new Auto_1(m_robotDrive, m_robotArm, m_robotScorer);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_driverController2 = new XboxController(OIConstants.kDriver2ControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

  //  m_robotArm.setDefaultCommand(new RunCommand(
    //    () -> m_robotArm.setSpeed(
      //      m_driverController2.getLeftY()), m_robotArm));


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */

   
  private void configureButtonBindings() {
    //Controller 1: Set wheels in X
    new JoystickButton(m_driverController, Button.kSquare.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    //Controller 1: Reset Gyro heading
    new JoystickButton(m_driverController, Button.kTriangle.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

            //Xbox Controller 1, holding L2 button turns shooter on
    new JoystickButton(m_driverController, Button.kL1.value)
        .whileTrue(new StartEndCommand(
                () -> m_robotScorer.shooting(),
                () -> m_robotScorer.stopShooting(),
                  m_robotScorer));

            //Xbox Controller 1, holding R2 button turns intake on
            //(Get shooter to speed, then intake to shoot)
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new StartEndCommand(
            () -> m_robotScorer.intake(),
            () -> m_robotScorer.intakeStop(),
            m_robotScorer));
            //Xbox Controller 1, R1 will roll intake back
    new JoystickButton(m_driverController, Button.kCircle.value)
        .whileTrue(new StartEndCommand(
            () -> m_robotScorer.intakeBack(),
            () -> m_robotScorer.intakeStop(),
            m_robotScorer));

//Additional buttons here for setting arm position to specific setPoints.
//Will pass values to the "setReference" in the Arm subsystem for PID loop to move to setPoints
//Driver controller 2 will set arm
/*
new JoystickButton(m_driverController2, Button.kTriangle.value)
    .whileTrue(new StartEndCommand(
        () -> m_robotArm.doSomethingPlease(),
        () -> m_robotArm.stopSomethingPlease(),
        m_robotArm));
*/

//Arm to Ground pickup A
new JoystickButton(m_driverController2, 1)
    .onTrue(new RunCommand(
        () -> m_robotArm.setPosition(154),
        m_robotArm ));

//Arm to Shooting point X

new JoystickButton(m_driverController2, 3)
    .onTrue(new RunCommand(
        () -> m_robotArm.setPosition(175),
        m_robotArm));

//Arm to Amp scoring point B
new JoystickButton(m_driverController2, 2)
    .onTrue(new RunCommand(
        () -> m_robotArm.setPosition(235),
        m_robotArm));

//Arm to hook set point START
new JoystickButton(m_driverController2, 8)
    .onTrue(new RunCommand(
        () -> m_robotArm.setPosition(235),
        m_robotArm));
//Arm to hook release point BACK
new JoystickButton(m_driverController2, 7)
    .onTrue(new RunCommand(
        () -> m_robotArm.setPosition(215),
        m_robotArm));

//Engage climber motor  Y
new JoystickButton(m_driverController2, 4)
    .whileTrue(new StartEndCommand(
        () -> m_robotClimber.goUp(),
        () -> m_robotClimber.stop(),
        m_robotClimber));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return m_auto;
        
    
  }
}
