package frc.robot.util.Tests;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Touchboard;

public class tbProxiedCommandTests {
    
    int abIndex = 0;

    public tbProxiedCommandTests() {
        // Test code here
        Touchboard.bindActionButton("abTest", ()-> Commands.run(() -> {

            System.out.println("abTest pressed" + abIndex);
            abIndex++;
        }));

        Touchboard.bindToggleButton("togTest", () -> Commands.run(() -> {

            System.out.println("togTest pressed" + abIndex);
            abIndex++;
        }));

        Touchboard.bindOneShotButton("oneShotTest", () -> Commands.run(() -> {

            System.out.println("oneShotTest pressed" + abIndex);
            abIndex++;
        }));

        Touchboard.bindAxis("axisTest", () -> Commands.run(() -> {
            double axisValue = Touchboard.getDoubleValue("axisTest");
            System.out.println("axisTest value: " + axisValue);
        }));

        Touchboard.bindNumberComponent("ncTest", () -> Commands.run(() -> {
            double ncValue = Touchboard.getDoubleValue("ncTest");
            System.out.println("ncTest value: " + ncValue);
        }));

        Touchboard.bindDropdown("dropdownTest", () -> Commands.run(() -> {
            String dropdownValue = Touchboard.getStringValue("dropdownTest");
            System.out.println("dropdownTest value: " + dropdownValue);
        }));

        Touchboard.bindOptGroup("optGroupTest", () -> Commands.run(() -> {
            String optGroupValue = Touchboard.getStringValue("optGroupTest");
            System.out.println("optGroupTest value: " + optGroupValue);
        }));

    }
}
