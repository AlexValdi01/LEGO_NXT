import lejos.nxt.Motor;
import lejos.robotics.RegulatedMotor;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.LightSensor;
import lejos.nxt.SensorPort;
import lejos.util.Delay;

public class Trabajo_PSETR {
    public static void main(String[] args) {
        // Configuracion del sensor de luminosidad
        LightSensor lightSensor1 = new LightSensor(SensorPort.S1);
        LightSensor lightSensor2 = new LightSensor(SensorPort.S4);

        // Constantes de control
        int maxSpeed = 250; // Velocidad maxima de los motores en grados por segundo
        int limit = 40; // Umbral para detectar la linea negra
        int steerAngle = 75; // Angulo maximo de giro de las ruedas delanteras

        // Variables de control
        boolean curveDetected = false; // Indica si se ha detectado una curva

        // Configuracion de los motores
        RegulatedMotor motorLeft = Motor.C;
        RegulatedMotor motorRight = Motor.A;
        RegulatedMotor motorSteer = Motor.B;

        // Inicializacion de los motores
        motorLeft.setSpeed(maxSpeed);
        motorRight.setSpeed(maxSpeed);
        motorSteer.setSpeed(200);

        while (!Button.ESCAPE.isDown()) {
            // Leer el valor del sensor de luminosidad
            int sensorIzquierdo = lightSensor1.getLightValue();
            int sensorDerecho = lightSensor2.getLightValue();

            if ((sensorIzquierdo > limit) && (sensorDerecho > limit)) { // Si ningun sensor detecta la linea, sigue recto
                motorLeft.backward();
                motorRight.backward();
                motorSteer.rotateTo(0);
                curveDetected = false;
            } else {
                // Ha detectado la linea
                motorLeft.setSpeed(maxSpeed / 4);
                motorRight.setSpeed(maxSpeed / 4);

                if (!curveDetected) {
                    if (sensorDerecho <= limit) { // Si el sensor derecho detecta la linea, gira hacia la derecha
                        motorSteer.rotate(steerAngle); // Girar a la derecha
                    } else if (sensorIzquierdo <= limit) { // Si el sensor izquierdo detecta la linea, gira hacia la izquierda
                        motorSteer.rotate(-steerAngle); // Girar a la izquierda
                    }
                    curveDetected = true;
                } else {
                    // Realizar el giro completo ajustando la duracion del giro en funcion de la distancia recorrida
                    int distance = Math.abs(motorSteer.getTachoCount());
                    if (distance >= 2 * steerAngle) {
                        motorSteer.rotateTo(0);
                        curveDetected = false;
                        Delay.msDelay(250); // Esperar un segundo antes de seguir recto
                    }
                }
            }

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            // Mostrar el valor de luminosidad y el color en la pantalla
            LCD.clear();
            LCD.drawString("Valor Izq: " + sensorIzquierdo, 0, 0);
            LCD.drawString("Valor Drc: " + sensorDerecho, 0, 2);
        }

        // Detener los motores
        motorLeft.flt();
        motorRight.flt();
    }
}
