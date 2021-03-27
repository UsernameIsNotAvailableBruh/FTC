package developing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestOdometry")
public class TestOdometry extends OpMode {
    TestRobot bot = new TestRobot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();

    Optimizer optimizer = new Optimizer();

    AutoModuleThread autoModuleThread = new AutoModuleThread();


    @Override
    public void init() {
        bot.init(hardwareMap);
    }

    @Override
    public void loop() {

        if(!optimizer.show) {
            bot.updateOdometry();
            optimizer.update();
        }else{
            telemetry.addData("avgDeltaTime", optimizer.avgDeltaTime);
            telemetry.update();
        }

        if(gamepad1.a && !optimizer.show){
            optimizer.show();
        }
        telemetry = telemetryHandler.addOdometry(telemetry, bot);
        telemetry = telemetryHandler.addAngularPosition(telemetry, bot);
        telemetry.update();



    }

    @Override
    public void stop() { }
}
