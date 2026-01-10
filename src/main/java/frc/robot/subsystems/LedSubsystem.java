package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedColors;

public class LedSubsystem extends SubsystemBase {
  //Creates a new subsystem for the LEDs

  Spark blinkin;
  boolean discoOn = false;

  public LedSubsystem() {
    blinkin = new Spark(0);
  }
  
  public void setLedColor(double color) {
    blinkin.set(color);
  }

  @Override
  public void periodic() {

    
    if (discoOn) {
      setRandomColor();
    }
  }

  public void setRandomColor() {
    double[] RandomLedColors = {LedColors.Hot_Pink, LedColors.Red, LedColors.Orange, LedColors.Yellow, LedColors.Green, 
      LedColors.Aqua, LedColors.Dark_Blue, LedColors.Blue, LedColors.Violet, LedColors.White, LedColors.Black};

    int randomNum = (int)(Math.random() * RandomLedColors.length);
    double randomColor = RandomLedColors[randomNum];
    setLedColor(randomColor);
  }

  public void setInitialColor() {
    if (isRedSide()) setLedColor(LedColors.Red);
    else setLedColor(LedColors.Blue);
  }

  public void ChangeLedColor (int num) {
    double[] SetLedColors = {LedColors.Green, LedColors.Red, LedColors.Lawn_Green, LedColors.Blue_Violet, LedColors.White};

    discoOn = false;

    if (num >= 0 && num <= 4) blinkin.set(SetLedColors[num]);
    else setInitialColor();
  }

  public boolean isRedSide() {
    var alliance = DriverStation.getAlliance();
      if (alliance.isPresent())
      {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
  }

}