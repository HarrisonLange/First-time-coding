package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.;

import java.sql.Driver;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  CANSparkMax m_leftDrive = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax m_rightDrive = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax intake = new CANSparkMax(2, MotorType.kBrushed);
  CANSparkMax arm = new CANSparkMax(3, MotorType.kBrushless);
  DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  Joystick DriverController = new Joystick(0);
  Timer m_timer = new Timer();
  PIDController pid = new PIDController(kP, kI, dD)

  // Constants for arm (will need to tune)
  final double armHoldUp = 0.08;
  final double armHoldDown = 0.13;
  final double armTravel = 0.5;

  final double armTimeUp = 0.5;
  final double armTimeDown = 0.35;

  // Arm Varibles
  boolean armUp = true;
  boolean burstMode = false;
  double lastBurstTime = 0;

  // Auto Controls
  double autoStart = 0;
  boolean Auto = false;




    // This function is run when the robot is first started up

  @Override
  public void robotInit() {

    // invert one side of the drivetrain
    m_rightDrive.setInverted(true);

    // Set drive motors to coast
    m_rightDrive.setIdleMode(IdleMode.kCoast);
    m_leftDrive.setIdleMode(IdleMode.kCoast);

    // incase I need to invert the lift arm
    arm.setInverted(false);

    // Set the arm motor to break
    arm.setIdleMode(IdleMode.kBrake);
  }

  // This function is run once each time the robot enters autonomous mode.
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // This controls the drive
    m_robotDrive.arcadeDrive(DriverController.getY(), DriverController.getX());

    // Intake Controls
    if (DriverController.getRawButton(5)) {
      intake.set(1);
    }
    else if (DriverController.getRawButton(7)) {
      intake.set(-1);
    }
    else{
      intake.set(0);
    }

    //Arm controls
    if(armUp){
      if (Timer.getFPGATimestamp() - lastBurstTime < armTimeUp) {
        arm.set(armTravel);
      }
        else {
          arm.set(armHoldUp);
        }
    }
    else{
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeUp){
        arm.set(-armTravel);
      }
      else{
        arm.set(-armHoldDown);
      }
    }

    if(DriverController.getRawButton(6) && !armUp){
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = true ;
    }
    else if(DriverController.getRawButton(8) && armUp){
      lastBurstTime = Timer.getFPGATimestamp();
      armUp= false;

    }
    
  }

  @Override
  public void disabledInit() {
        // On disable set all values to zero
    // This is to prevent motors going to their privous speed on startup
    m_leftDrive.set(0);
    m_rightDrive.set(0);
    arm.set(0);
    intake.set(0);
  }
}