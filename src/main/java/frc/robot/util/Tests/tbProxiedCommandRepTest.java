package frc.robot.util.Tests;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Touchboard;

public class tbProxiedCommandRepTest {
    
    int abIndex = 0;

    public tbProxiedCommandRepTest() {
        // Test code here
        Touchboard.bindActionButton("abTest", ()-> Commands.runOnce(() -> {

            System.out.println("abTest pressed" + abIndex);
            abIndex++;
        }));

        Touchboard.bindToggleButton("togTest", () -> Commands.runOnce(() -> {

            System.out.println("togTest pressed" + abIndex);
            abIndex++;
        }));

        Touchboard.bindOneShotButton("oneShotTest", () -> Commands.runOnce(() -> {

            System.out.println("oneShotTest pressed" + abIndex);
            abIndex++;
        }));

        Touchboard.bindAxis("axisTest", () -> Commands.runOnce(() -> {
            double axisValue = Touchboard.getDoubleValue("axisTest");
            System.out.println("axisTest value: " + axisValue);
        }));

        Touchboard.bindNumberComponent("ncTest", () -> Commands.runOnce(() -> {
            double ncValue = Touchboard.getDoubleValue("ncTest");
            System.out.println("ncTest value: " + ncValue);
        }));

        Touchboard.bindDropdown("dropdownTest", () -> Commands.runOnce(() -> {
            String dropdownValue = Touchboard.getStringValue("dropdownTest");
            System.out.println("dropdownTest value: " + dropdownValue);
        }));

        Touchboard.bindOptGroup("optGroupTest", () -> Commands.runOnce(() -> {
            String optGroupValue = Touchboard.getStringValue("optGroupTest");
            System.out.println("optGroupTest value: " + optGroupValue);
        }));

    }
}
