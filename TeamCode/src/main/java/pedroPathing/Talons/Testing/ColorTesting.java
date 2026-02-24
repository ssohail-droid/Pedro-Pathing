//package pedroPathing.Talons.Testing;
//
//import com.qualcomm.hardware.rev.RevColorSensorV3;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//
//import org.firstinspires.ftc.robotcore.external.JavaUtil;
//
//import pedroPathing.Talons.General.ColorSensed;
//
//@TeleOp
//public class ColorTesting extends LinearOpMode {
//    RevColorSensorV3 color;
//    double saturationR, hueR, saturationLimitBlue, saturationLimitRed;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        color = hardwareMap.get(RevColorSensorV3.class, "sensor_color_right");
//
//        waitForStart();
//
//        while (opModeIsActive()){
//
//            detectColor();
//
//            if (gamepad1.dpad_up && !dpad){
//                satu += .03;
//            }
//            else if (gamepad1.dpad_down && !dpad){
//                servoPos -=.03;
//            }
//            dpad = gamepad1.dpad_down || gamepad1.dpad_up;
//
//
//        }
//
//    }
//
//
//    public ColorSensed detectColor(){
//
//
//        saturationR = JavaUtil.rgbToSaturation(color.red(), color.green(), color.blue());
//        hueR = JavaUtil.rgbToHue(color.red(), color.green(), color.blue());
//
//        ColorSensed  sensedColor =  (hueR > 160 && saturationR < .3) ? ColorSensed.PURPLE : ((hueR < 150 && saturationR > .65) ? ColorSensed.GREEN : ColorSensed.INCONCLUSIVE);
//
//        return sensedColor;
//    }
//}
