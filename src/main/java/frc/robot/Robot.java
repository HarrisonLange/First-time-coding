package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import java.sql.Driver;

import javax.swing.plaf.TreeUI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  static double kDt = 0.02;
   // Constants for arm (will need to tune)
   final double armHoldUp = 0.08;
   final double armHoldDown = 0.13;
   final double armTravel = 0.5;
 
   final double armTimeUp = 0.5;
   final double armTimeDown = 0.35;
  
   // Arm Motion Profile
   double MaxV = 1.75;
   double MaxA = 0.75;
 
   // Arm Varibles
   boolean armUp = true;
   boolean burstMode = false;
   double lastBurstTime = 0;
 
   // Auto Controls
   double autoStart = 0;
   boolean Auto = false;

  // Motor setups
    CANSparkMax m_leftDrive = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax m_rightDrive = new CANSparkMax(2, MotorType.kBrushless);
    CANSparkMax intake = new CANSparkMax(3, MotorType.kBrushed);
    CANSparkMax arm = new CANSparkMax(4, MotorType.kBrushless);
    
    // Motion Setups
    DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
    Joystick DriverController = new Joystick(0);
    Timer m_timer = new Timer();
    final TrapezoidProfile.Constraints Arm_Motion =
      new TrapezoidProfile.Constraints((MaxV), MaxA);
    TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    SparkMaxPIDController m_PidController;
    RelativeEncoder m_eEncoder;

    // This function is run when the robot is first started up

  @Override
  public void robotInit() {

    // invert one side of the drivetrain
    m_rightDrive.setInverted(true);

    // Set drive motors to coast
    m_rightDrive.setIdleMode(IdleMode.kCoast);
    m_rightDrive.burnFlash();
    m_leftDrive.setIdleMode(IdleMode.kCoast);
    m_leftDrive.burnFlash();

    // incase I need to invert the lift arm
    arm.setInverted(false);

    // Set the arm motor to break
    arm.setIdleMode(IdleMode.kBrake);
    arm.burnFlash();

    //Inizalize PID
    m_PidController = arm.getPIDController();
    m_eEncoder = arm.getEncoder();
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
    // teleop varibles
      //Drive Varibles
      double xspeed = DriverController.getY();
      double zrotation = DriverController.getX();
      boolean isQuickturn = true;

      // Feedforward Varibles
      /** 
      double arm_ffks = SmartDashboard.getNumber("Arm FF kS", 0);
      double arm_ffkv = SmartDashboard.getNumber("Arm FF kV", 0);
      final SimpleMotorFeedforward m_Feedforward = new SimpleMotorFeedforward(arm_ffks, arm_ffkv);
      */
      //Intake Varibles
      
    // Smart dashboard data
  SmartDashboard.putBoolean("Arm Up", armUp);
  SmartDashboard.putNumber("Speed", DriverController.getY());
  SmartDashboard.putNumber("Turn", DriverController.getX());
  SmartDashboard.putNumber("Encoder", m_eEncoder.getPosition());
  double arm_kP = SmartDashboard.getNumber("Arm kP", 0.1);
  double arm_kI = SmartDashboard.getNumber("Arm kI", 1e-4);
  double arm_kD = SmartDashboard.getNumber("Arm kD", 1);
  double arm_iz = SmartDashboard.getNumber("Arm I zone", 0);
  double arm_ff = SmartDashboard.getNumber("Feed Forward", 0);
  double arm_min = SmartDashboard.getNumber("Arm Min Ouitput", 0);
  double arm_max = SmartDashboard.getNumber("Arm Max Output", 0);
  double arm_up = SmartDashboard.getNumber("Arm Up Position", 0);
  double arm_dp = SmartDashboard.getNumber("Arm Down Position", 0);

    //PID Vars
    double p = 0;
    double i = 0;
    double d = 0;
    double iz = 0;
    double ff = 0;
    double min = 0;
    double max = 0;

    if((p != arm_kP)) { m_PidController.setP(arm_kP); arm_kP = p;}
    if((i != arm_kI)) { m_PidController.setI(arm_kI); arm_kI = i;}
    if((d != arm_kD)) { m_PidController.setD(arm_kD); arm_kD = d;}
    if((iz != arm_iz)) { m_PidController.setIZone(arm_iz); arm_iz = iz;}
    if((ff != arm_ff)) { m_PidController.setFF(arm_ff); arm_ff = ff;}
    if((max != arm_max) || (min != arm_min)){
      m_PidController.setOutputRange(arm_min, arm_max);
      arm_min = min; arm_max = max;
    }


    // This controls the drive
    m_robotDrive.curvatureDrive(xspeed * xspeed, zrotation * zrotation, isQuickturn);

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

    // Arm Motion control
      if (DriverController.getRawButton(6)) {
        m_goal = new TrapezoidProfile.State(arm_up, 0);
      }
      else if (DriverController.getRawButton(8)) {
        m_goal = new TrapezoidProfile.State(arm_dp, 0);
      }

      // Create the motion profile with given max v and a
      var profile = new TrapezoidProfile(Arm_Motion, m_goal, m_setpoint);

      // Retrieve the setpoint for the next timestep
      m_setpoint = profile.calculate(kDt);

      // Send setpoint to PID controller
      m_PidController.setReference(m_setpoint.position, ControlType.kPosition);

    //Arm controls Time Based
    /** 
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
    */
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