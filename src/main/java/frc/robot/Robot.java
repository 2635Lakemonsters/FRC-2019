/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI oi;
  public static DriveSubsystem driveSubsystem;
  public static Elevator elevator;
  public static Flower flower;
  public static Switcher switcher;

  SPathLeftCmd sPathLeftCmd;
  DriveCommand driveCommand;
  ToggleFlowerExtendCommand extenderCommand;
  ToggleFlowerCommand flowerCommand;
  ReverseCommand reverseCommand;
  EncoderResetCommand encoderResetCommand;
  SwitcherCommand switcherCommand;
  SwitcherEncoderResetCommand switcherEncoderResetCommand;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  
  @Override
  public void robotInit() {
    
    oi = new OI();
    driveSubsystem = new DriveSubsystem();
    elevator = new Elevator();
    flower = new Flower();
    switcher = new Switcher();

    sPathLeftCmd = new SPathLeftCmd();
    driveCommand = new DriveCommand();
    extenderCommand = new ToggleFlowerExtendCommand();
    reverseCommand = new ReverseCommand();
    encoderResetCommand = new EncoderResetCommand(5);
    switcherCommand = new SwitcherCommand();
    switcherEncoderResetCommand = new SwitcherEncoderResetCommand(5);

    //m_chooser.setDefaultOption("Default Auto", new PathTestCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    oi.sPathButton.whenPressed(sPathLeftCmd);
    oi.grabberExtendButton.whenPressed(extenderCommand);
    oi.flowerButtonL.whenPressed(flowerCommand);
    oi.flowerButtonR.whenPressed(flowerCommand);
    oi.reverseButton.whenPressed(reverseCommand);
    oi.encoderResetButton.whenPressed(encoderResetCommand);
    oi.switcherButton.toggleWhenPressed(switcherCommand);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    if (driveCommand != null && driveCommand.isRunning())
		{
			driveCommand.cancel();
		}
    if(autoHappened){
      driveSubsystem.endPath();
      autoHappened = false;
    }
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
boolean autoHappened = false;

  @Override
  public void autonomousInit() {

    //-------EXPERIMENTAL PATH WEAVER CODE-----------
    switcherEncoderResetCommand.start();

  //  driveSubsystem.PathAutoInit();
    //---------------------------------------------
    
    //m_autonomousCommand = m_chooser.getSelected();
    

    //drive.motionMagic("aaa");

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      //m_autonomousCommand.start();
      }
      driveSubsystem.bullyOff();
      autoHappened = true;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    //driveSubsystem.bullyOff();
  }

  boolean boolistatus;
  @Override
  public void teleopInit() {
    switcherEncoderResetCommand.start();   //remove for real competitions

		if (driveCommand != null) {
			driveCommand.start();
		}
    
    //-------EXPERIMENTAL PATH WEAVER CODE-----------
    // if(autoHappened){
    //   driveSubsystem.endPath();
    //   autoHappened = false;
    // }
    //--------------------------------------------

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    driveSubsystem.bullyOff();
    boolistatus = false;

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    
    //System.out.println("Right: " + driveSubsystem.FLMotor.getSelectedSensorPosition(0));
    

    if(oi.leftJoy.getRawButton(3) && boolistatus == false){
      driveSubsystem.switchDriveMode();
      boolistatus = true;
    }else if(!oi.leftJoy.getRawButton(3) && boolistatus == true){
      boolistatus = false;
    }
    //System.out.println("Left Position: " + driveSubsystem.FLMotor.getSelectedSensorPosition(0));
    //System.out.println("Right Position: " + driveSubsystem.FRMotor.getSelectedSensorPosition(0));

    //System.out.println("Front Right Motor Current: " + driveSubsystem.FRMotor.getOutputCurrent());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
