package frc.robot.util.Tests;

// import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Touchboard;

public class tbRunnableTests {
    int abIndex = 0;
    
    public tbRunnableTests() {
        // Test code here

         Touchboard.bindActionButton("abTest", () -> {

            System.out.println("abTest pressed" + abIndex);
            abIndex++;
        });

        Touchboard.bindToggleButton("togTest", () -> {

            System.out.println("togTest pressed" + abIndex);
            abIndex++;
        });

        Touchboard.bindOneShotButton("oneShotTest", () -> {

            System.out.println("oneShotTest pressed" + abIndex);
            abIndex++;
        });

        Touchboard.bindAxis("axisTest", () -> {
            double axisValue = Touchboard.getDoubleValue("axisTest");
            System.out.println("axisTest value: " + axisValue);
        });

        Touchboard.bindNumberComponent("ncTest", () -> {
            double ncValue = Touchboard.getDoubleValue("ncTest");
            System.out.println("ncTest value: " + ncValue);
        });

        Touchboard.bindDropdown("dropdownTest", () -> {
            String dropdownValue = Touchboard.getStringValue("dropdownTest");
            System.out.println("dropdownTest value: " + dropdownValue);
        });

        Touchboard.bindOptGroup("optGroupTest", () -> {
            String optGroupValue = Touchboard.getStringValue("optGroupTest");
            System.out.println("optGroupTest value: " + optGroupValue);
        });

    }
}
