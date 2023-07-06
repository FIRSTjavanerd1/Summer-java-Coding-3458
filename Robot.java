// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
// these are here as things that exist in the code that we can call on
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

 // these are our autonomous programms/actions
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private MecanumDrive m_drive;

  private Joystick m_driver;
  private Joystick m_operater;
//these are our PID values
  private double p = 0.0005;
  private double i = 0.05;
  private double d = 0.0;
//positions for the claw
  private double stowed = 0.0;
  private double middle = 540; 
  private double high = 820;
  private double ground = 600;
  private double shelf = 1050;
//our motors
  private CANSparkMax frontleftdrive = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax rearleftdrive = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax frontrightdrive = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax rearrightdrive = new CANSparkMax(4, MotorType.kBrushless);

  private WPI_VictorSPX wrist = new WPI_VictorSPX(5);
//our pneumatics
  private final DoubleSolenoid BottomArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0,1);
  private final DoubleSolenoid TopArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2,3);
  private final DoubleSolenoid Claw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4,5);
  private final DoubleSolenoid Brakes = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6,7);

  private Encoder encoder = new Encoder (3, 2);

  private final PIDController wristController = new PIDController(p, i, d);

  private final Timer m_timer = new Timer();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   //This is robot initialization, everything here happens one time when the robot is booted up
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
//sets the wheels to all be spinning in the same direction (same forward)
    rearrightdrive.setInverted(true);
    frontrightdrive.setInverted(true);

    m_drive = new MecanumDrive(frontleftdrive, rearleftdrive,frontrightdrive,rearrightdrive);

    m_driver = new Joystick( 0);
    m_operater = new Joystick( 1);

    encoder.setReverseDirection(true);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */


  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
//this sets and resets a timer so that the robot knows when and in what order to do things during auto
    encoder.reset();
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
      //for the first 0.4 seconds the arm goes to top position
      
      if(m_timer.get() < 0.4){
        TopArm.set(Value.kForward);
        BottomArm.set(Value.kForward);

      }
// wrist moves up for 2.6 seconds
      if(m_timer.get() > 0.4 && m_timer.get() < 3.0){
        wrist.set(wristController.calculate(encoder.get(),high));
      }
// drives forward for 0.5 seconds
      if(m_timer.get() > 3.0 && m_timer.get() < 3.5){
        m_drive. driveCartesian( 0,  0.25,  0);
        wrist.set(0.0);

      }
//opens the claw
      if(m_timer.get() > 3.5 && m_timer.get() < 3.6){
        Claw.set(Value.kForward);
      }
//stows the wrist
      if(m_timer.get() > 3.6 && m_timer.get() < 4.6){
        wrist.set(wristController.calculate(encoder.get(),stowed));
      }
      //moves backwards for 2 seconds
      if(m_timer.get() > 4.6 && m_timer.get() < 6.6){
        m_drive. driveCartesian( -0,  -0.25,  0);
      }
      // drops the arm to the bottom position
      if(m_timer.get() > 6.6 && m_timer.get() < 9.6){
        TopArm.set(Value.kReverse);
        BottomArm.set(Value.kReverse);
      }
      // drives backwards for 3.3 seconds
      if(m_timer.get() > 9.6 && m_timer.get() < 12.9){
        m_drive. driveCartesian( -0,  -0.33,  0);
      }



        // Put custom auto code here 
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

     Brakes.set(Value.kReverse);

    wristController.setSetpoint(stowed);
  }

  /* This lets us drive */
  @Override
  public void teleopPeriodic() {
m_drive. driveCartesian(m_driver.getRawAxis( 0), -m_driver. getRawAxis(1), m_driver.getRawAxis(2));
//this lets us open the claw when button pressed
if(m_operater.getRawButtonPressed(5)){
  Claw.set(Value.kForward);
  //this lets us close the claw when button pressed
}
if(m_operater.getRawButtonPressed(6)){
  Claw.set(Value.kReverse);
}

//this puts the brakes down
if(m_driver.getRawButtonPressed(2)){
  Brakes.set(Value.kReverse);
}
//this lifts the brakes up
if(m_driver.getRawButtonPressed(3)){
  Brakes.set(Value.kForward);
}
//this stows the wrist/claw and lowers the arm all the way down
if(m_operater.getRawButtonReleased(1) || m_operater.getRawButtonReleased(3) ||m_operater.getRawButtonReleased(2) || m_operater.getRawButtonReleased(4)){
  wristController.setSetpoint(stowed);
  TopArm.set(Value.kReverse);
  BottomArm.set(Value.kReverse);
  //this lifts the arms all the way up, as well as the wrist
}
if(m_operater.getRawButtonPressed(4)){
  wristController.setSetpoint(high);
  TopArm.set(Value.kForward);
  BottomArm.set(Value.kForward);
}
// this puts them to the middle, firing one pneumatic
if(m_operater.getRawButtonPressed(3)){
  wristController.setSetpoint(middle);
TopArm.set(Value.kForward);
BottomArm.set(Value.kReverse);
}
//sets them both to the top position, and puts the claw into the shelf position
if(m_operater.getRawButtonPressed(1)){
  wristController.setSetpoint(shelf);
  TopArm.set(Value.kForward);
  BottomArm.set(Value.kForward);
}
// this makes it so when you press the compass point button on the operator controller it will move up or down
if(m_operater.getPOV() == 0){
  wrist.set(0.4);

}
if(m_operater.getPOV() == 180){
  wrist.set(-0.4);
}
// this makes sure that the wrist is within 60 encoder ticks of it's target, moving up or down at half speed if necesary
else if(wristController.getSetpoint() - encoder.get() > 30 || wristController.getSetpoint() - encoder.get() < -30){
wrist.set(wristController.calculate(encoder.get()) * 0.5);
  }
else {
  wrist.set(0.0);
}

if(m_operater.getRawButtonPressed(2)){
  TopArm.set(Value.kReverse);
  BottomArm.set(Value.kReverse);
  wristController.setSetpoint(ground);
}



  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
