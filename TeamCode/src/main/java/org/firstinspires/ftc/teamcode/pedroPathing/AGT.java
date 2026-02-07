package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.util.Range;
import java.util.List;

@Autonomous(name="AGT")
public class AGT extends OpMode {

    GoBildaPinpointDriver pinpoint;
    private DcMotorEx M_lanzador1, M_lanzador2;
    private Servo Lanzador_I, Lanzador_D, Levantador, Direccion, Licuadora;

    private ColorSensor sensor1;
    private TouchSensor sensorM;

    private Limelight3A limaluz;

    double goalY = -39; //cm
    double goalX = 333; //cm

    double H2 = 0.85675;


    double Thetai = -40;
    double Theta0 = 0;

    double V0 = 0;

    double xb;
    double v;

    double g = 9.77;

    double direccion = 0;
    double velM;

    // Estos dos los calculé (son medidas fisicas)
    double radioLanzador = 0.05;
    double ticksPorRevolucionMotor = 28;

    int window = 7;
    int[] hsvHist = new int[window];
    int hsvIdx = 0;

    double ruido = 850.0;
    double saturacion = 0.10;
    double oscuro = 0.04;

    float verdeMin = 140f;
    float verdeMax = 175f;

    float moradoMin = 195f;
    float moradoMax = 255f;


    enum Modo {A, X, B}

    String slotA = "Nada";
    String slotB = "Nada";
    String slotX = "Nada";

    double posA = 0.473;
    double posB = 0.95;
    double posX = 0.0;

    double Kp = 0.002;
    double Ki = 0.0000;
    double Kd = 0.1;

    boolean hayTag = false;
    double tx = 0.0;
    long stale = -1;
    int fidCount = 0;

    double correccion = 0.0;

    double currentAngle = 0.0;
    boolean tieneObjetivo = false;

    double errorAnterior = 0;
    double integral = 0;

    double referencia = 0;
    ArrayList<Boolean> lista_MOR = new ArrayList<Boolean>();

    private String[] data = {"", "", ""};

    Modo modo = Modo.A;

    private Limelight3A limelight;

    boolean botonL_actual = false;
    boolean botonL_anterior = false;

    int contador = 0;

    int estado_lanzar = 0;
    boolean estado_lanzador = false;

    long inicio_lanzador = 0;

    boolean lastPiecePresent = false;

    int ID = -2;

    private static final int ESPERAR_ALINEACION = 0;
    private static final int INICIO_A_RECOLECCION = 1;
    private static final int RECOLECCION = 2;
    private static final int RECOLECCION1_A_LANZAMIENTO1 = 3;
    private static final int LANZAMIENTO1_A_RECOLECCION2 = 4;
    private static final int RECOLECCION2 = 5;
    private static final int RECOLECCION2_A_LANZAMIENTO2 = 6;
    private static final int LANZAMIENTO2_A_RECOLECCION3 = 7;
    private static final int RECOLECCION3 = 8;
    private static final int RECOLECCION3_A_LANZAMIENTO3 = 9;
    private static final int FIN = 10;
    private static final int DONE = 11;


    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private int estado = INICIO_A_RECOLECCION;


    // inicioARecoleccion1
    private final Pose posicionInicio = new Pose(56, 8, Math.toRadians(90));
    private final Pose control1 = new Pose(55.324717285945084, 29.25848142164781);
    private final Pose posicionRecoleccion1 = new Pose(42.50726978998386
            , 35.76736672051696, Math.toRadians(180));

    // recoleccion1
    private final Pose posicionFinRecoleccion1 = new Pose(24.827140549273018, 35.76736672051696);

    // recoleccion1Atiro1
    private final Pose posicionRecoleccion1Atiro1 = new Pose(46.833602584814216
            , 15.558966074313398, Math.toRadians(90));

    // tiro1ARecoleccion2
    private final Pose posicionTiro1ARecoleccion2 = new Pose(42.68336025848143
            , 60.26009693053314, Math.toRadians(180));

    // recoleccion2
    private final Pose posicionFinRecoleccion2 = new Pose(24.02746365105009
            , 59.93053311793216, Math.toRadians(180));

    // recoleccion2Atiro2
    private final Pose posicionRecoleccion2Atiro2 = new Pose(61.424878836833614
            , 81.37964458804524, Math.toRadians(125));

    // tiro2ARecoleccion3
    private final Pose posicionTiro2ARecoleccion3 = new Pose(41.903069466882066
            , 84.10339256865913, Math.toRadians(180));
    private final Pose control2 = new Pose(51.035541195476576, 81.67043618739903);

    // recoleccion3
    private final Pose posicionFinRecoleccion3 = new Pose(27.553150242326338
            , 83.92245557350566, Math.toRadians(180));

    // recoleccion3Atiro3
    private final Pose posicionRecoleccion3Atiro3 = new Pose(43.953150242326316
            , 94.41680129240711, Math.toRadians(128));

    //fin
    private final Pose posicionFin = new Pose(24.169628432956372
            , 70.24555735056542, Math.toRadians(180));
    private final Pose control3 = new Pose(43.89095315024232, 70.88691437802909);

    private PathChain inicioARecoleccion1, recoleccion1, recoleccion1Atiro1, tiro1ARecoleccion2, recoleccion2,
            recoleccion2Atiro2, tiro2ARecoleccion3, recoleccion3, recoleccion3Atiro3, fin;

    public void buildPaths() {
        inicioARecoleccion1 = follower.pathBuilder()
                .addPath(new BezierCurve(posicionInicio, control1, posicionRecoleccion1))
                .setLinearHeadingInterpolation(posicionInicio.getHeading(), posicionRecoleccion1.getHeading())
                .build();
        recoleccion1 = follower.pathBuilder()
                .addPath(new BezierLine(posicionRecoleccion1, posicionFinRecoleccion1))
                .setLinearHeadingInterpolation(posicionRecoleccion1.getHeading(), posicionRecoleccion1.getHeading())
                .build();
        recoleccion1Atiro1 = follower.pathBuilder()
                .addPath(new BezierLine(posicionFinRecoleccion1, posicionRecoleccion1Atiro1))
                .setLinearHeadingInterpolation(posicionRecoleccion1.getHeading(),
                        posicionRecoleccion1Atiro1.getHeading())
                .build();
        tiro1ARecoleccion2 = follower.pathBuilder()
                .addPath(new BezierLine(posicionRecoleccion1Atiro1, posicionTiro1ARecoleccion2))
                .setLinearHeadingInterpolation(posicionRecoleccion1Atiro1.getHeading(),
                        posicionTiro1ARecoleccion2.getHeading())
                .build();
        recoleccion2 = follower.pathBuilder()
                .addPath(new BezierLine(posicionTiro1ARecoleccion2, posicionFinRecoleccion2))
                .setLinearHeadingInterpolation(posicionTiro1ARecoleccion2.getHeading(),
                        posicionFinRecoleccion2.getHeading())
                .build();
        recoleccion2Atiro2 = follower.pathBuilder()
                .addPath(new BezierLine(posicionFinRecoleccion2, posicionRecoleccion2Atiro2))
                .setLinearHeadingInterpolation(posicionFinRecoleccion2.getHeading(),
                        posicionRecoleccion2Atiro2.getHeading())
                .build();
        tiro2ARecoleccion3 = follower.pathBuilder()
                .addPath(new BezierCurve(posicionRecoleccion2Atiro2, control2, posicionTiro2ARecoleccion3))
                .setLinearHeadingInterpolation(posicionRecoleccion2Atiro2.getHeading(),
                        posicionTiro2ARecoleccion3.getHeading())
                .build();
        recoleccion3 = follower.pathBuilder()
                .addPath(new BezierLine(posicionTiro2ARecoleccion3, posicionFinRecoleccion3))
                .setLinearHeadingInterpolation(posicionTiro2ARecoleccion3.getHeading(),
                        posicionFinRecoleccion3.getHeading())
                .build();
        recoleccion3Atiro3 = follower.pathBuilder()
                .addPath(new BezierLine(posicionFinRecoleccion3, posicionRecoleccion3Atiro3))
                .setLinearHeadingInterpolation(posicionFinRecoleccion3.getHeading(),
                        posicionRecoleccion3Atiro3.getHeading())
                .build();
        fin = follower.pathBuilder()
                .addPath(new BezierCurve(posicionRecoleccion3Atiro3, control3, posicionFin))
                .setLinearHeadingInterpolation(posicionRecoleccion3Atiro3.getHeading(),
                        posicionFin.getHeading())
                .build();
    }

    @Override
    public void init() {

        M_lanzador1 = hardwareMap.get(DcMotorEx.class, "M_lanzador1");
        M_lanzador2 = hardwareMap.get(DcMotorEx.class, "M_lanzador2");

        M_lanzador1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        M_lanzador1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Lanzador_I = hardwareMap.get(Servo.class, "Lanzador_I");
        Lanzador_D = hardwareMap.get(Servo.class, "Lanzador_D");

        Direccion = hardwareMap.get(Servo.class, "Direccion");
        Licuadora = hardwareMap.get(Servo.class, "Licuadora");
        Levantador = hardwareMap.get(Servo.class, "Levantador");

        sensor1 = hardwareMap.get(ColorSensor.class, "sensor1");

        limaluz = hardwareMap.get(Limelight3A.class, "limaluz");
        limaluz.setPollRateHz(90);
        limaluz.pipelineSwitch(5);

        Direccion.setDirection(Servo.Direction.REVERSE);

        limaluz.start();

        LLResult result = limaluz.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> detections = result.getFiducialResults();

                for (LLResultTypes.FiducialResult tag : detections) {
                    ID = tag.getFiducialId();
                }
            }
        }

        if (ID == 21) {
            // verde, morado, morado
            lista_MOR.add(false);
            lista_MOR.add(true);
            lista_MOR.add(true);

        } else if (ID == 22) {
            // morado, verde, morado
            lista_MOR.add(true);
            lista_MOR.add(false);
            lista_MOR.add(true);


        } else if (ID == 23) {
            // morado, morado, verde
            lista_MOR.add(true);
            lista_MOR.add(true);
            lista_MOR.add(false);
        }

        limaluz.pipelineSwitch(6);


        Direccion.setPosition(0);

        Licuadora.setPosition(posX);
        modo = Modo.X;

        slotB = data[0];
        slotA = data[1];
        slotX = data[2];

        Levantador.setPosition(0.6);

        slotA = "Nada";
        slotB = "Nada";
        slotX = "Nada";

        inicio_lanzador = 0;
        estado_lanzar = 0;
        estado_lanzador = false;

        M_lanzador1.setVelocityPIDFCoefficients(105, 0.0, 0.1, 12.0);

        configurePinpoint();
        pinpoint.setPosition(new Pose2D(DistanceUnit.CM, 23, -142, AngleUnit.DEGREES, 0));
        pinpoint.update();

        estado = ESPERAR_ALINEACION;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(posicionInicio);
    }

    @Override
    public void start () {
        opModeTimer.resetTimer();
        setPathState(estado);
    }

    @Override
    public void loop () {

        pinpoint.update();

        double pos_Y = pinpoint.getPosY(DistanceUnit.CM);
        double pos_X = pinpoint.getPosX(DistanceUnit.CM);

        double deltaX = goalX - pos_X;
        double deltaY = goalY - pos_Y;

        xb = (Math.sqrt(deltaX*deltaX + deltaY*deltaY))/100;

        Theta0 = Math.atan((2*H2/xb) - Math.tan(Math.toRadians(Thetai)));

        V0 = ((1/Math.cos(Theta0))*Math.sqrt(g*xb/(Math.tan(Theta0)-Math.tan(Math.toRadians(Thetai)))));

        velM = (V0/(Math.PI*radioLanzador))*(ticksPorRevolucionMotor);
        direccion = ((68 - Math.toDegrees(Theta0))/38)*0.2;

        Direccion.setPosition(direccion);
        v = (int)(1436.44779*Math.pow(1.08097, xb));

        follower.update();
        statePathUpdate();
    }

    public void lanzamiento(boolean alineado) {
        v = (int) (1436.44779 * Math.pow(1.08097, xb));

        if (alineado) {
            M_lanzador1.setVelocity(v);
            estado_lanzar = 3;
            estado_lanzador = true;
            inicio_lanzador = System.nanoTime();
        } else {
            estado_lanzador = false;
            M_lanzador1.setVelocity(0);
        }

        if (estado_lanzador) {
            if (estado_lanzar == 3) {
                double velocidadActual = M_lanzador1.getVelocity();
                telemetry.addData("Vi velocidad Actual es", velocidadActual);
                if (Math.abs(velocidadActual - v) < 30) {
                    estado_lanzar = 2;
                    Levantador.setPosition(1.0);
                    inicio_lanzador = System.nanoTime();
                }

            } else if (estado_lanzar == 2) {
                if ((System.nanoTime() - inicio_lanzador) / 1_000_000 > 550) {
                    Levantador.setPosition(0.6);
                    estado_lanzar = 0;
                    estado_lanzador = false;

                    if (modo == Modo.X) slotX = "Nada";
                    else if (modo == Modo.A) slotA = "Nada";
                    else slotB = "Nada";
                }

            } else if (estado_lanzar == 1) {
                if ((System.nanoTime() - inicio_lanzador) / 1_000_000 > 2000) {
                    inicio_lanzador = System.nanoTime();
                    estado_lanzar = 2;
                    Levantador.setPosition(1.0);
                }

            } else {
                estado_lanzador = false;
                M_lanzador1.setVelocity(0);
            }
        }
    }

    public String identificar_color() {
        String detectado = "Nada";
        double rojo = sensor1.red();
        double verde = sensor1.green();
        double azul = sensor1.blue();
        int clase = 0; // 0 Nada, 1 Verde, 2 Purpura
        double suma = rojo + verde + azul;
        float H = 0f, S = 0f, Vv = 0f;
        if (suma >= ruido) {
            double rn = rojo / suma;
            double gn = verde / suma;
            double bn = azul / suma;
            int R = (int) Math.round(rn * 255.0);
            int G = (int) Math.round(gn * 255.0);
            int B = (int) Math.round(bn * 255.0);
            R = Range.clip(R, 0, 255);
            G = Range.clip(G, 0, 255);
            B = Range.clip(B, 0, 255);
            float[] hsv = new float[3];
            android.graphics.Color.RGBToHSV(R, G, B, hsv);
            H = hsv[0];
            S = hsv[1];
            Vv = hsv[2];
            if (S >= saturacion && Vv >= oscuro) {
                if (H >= verdeMin && H <= verdeMax) {
                    clase = 1;
                } else if (H >= moradoMin && H <= moradoMax) {
                    clase = 2;
                }
            }
        }
        hsvHist[hsvIdx] = clase;
        hsvIdx = (hsvIdx + 1) % window;
        int c0 = 0, c1 = 0, c2 = 0;
        for (int i = 0; i < window; i++) {
            int v = hsvHist[i];
            if (v == 1) c1++;
            else if (v == 2) c2++;
            else c0++;
        }
        int estable;
        if (c1 > c2 && c1 >= 3) estable = 1;
        else if (c2 > c1 && c2 >= 3) estable = 2;
        else estable = 0;
        boolean hayVerde = (estable == 1);
        boolean hayPurpura = (estable == 2);
        boolean siNoHayAlgo = (estable == 0);
        detectado = "Nada";
        if (hayVerde) {
            detectado = "Verde";
        } else if (hayPurpura) {
            detectado = "Purpura";
        } else {
            detectado = "Nada";
        }
        return detectado;
    }

    public void recogedor_3() {

        if (modo == Modo.X && "Nada".equals(slotX)) {
            slotX = identificar_color();
            Licuadora.setPosition(posA);
            modo = Modo.A;
        } else if (modo == Modo.A && "Nada".equals(slotA)) {
            slotA = identificar_color();
            Licuadora.setPosition(posB);
            modo = Modo.B;

        } else if (modo == Modo.B && "Nada".equals(slotB)) {
            slotB = identificar_color();

            Licuadora.setPosition(posX);
            modo = Modo.X;
        }
    }


    public void licuadora_mov(int i) {
        if (i != 3) {
            boolean pedirPurpura = lista_MOR.get(i);

            if (!estado_lanzador) {

                String objetivo = pedirPurpura ? "Purpura" : "Verde";

                boolean existe = objetivo.equals(slotX) || objetivo.equals(slotA) || objetivo.equals(slotB);

                if (!existe) {
                    estado_lanzador = false;
                    estado_lanzar = 0;
                } else {
                    estado_lanzador = true;


// Si el objetivo ya está en el modo actual, lanza directo
                    if ((modo == Modo.X && objetivo.equals(slotX)) ||
                            (modo == Modo.A && objetivo.equals(slotA)) ||
                            (modo == Modo.B && objetivo.equals(slotB))) {

                        estado_lanzar = 3;
                        estado_lanzador = true;
                        inicio_lanzador = System.nanoTime();

                    } else {
// Si no está en el slot actual, rota hacia el slot que lo tiene de forma óptima
                        if (modo == Modo.X) {
                            if (objetivo.equals(slotA) && !objetivo.equals(slotB) ||
                                    (objetivo.equals(slotA) && objetivo.equals(slotB))) {
                                Licuadora.setPosition(posA);
                                modo = Modo.A;
                            } else if (!objetivo.equals(slotA) && objetivo.equals(slotB)) {
                                Licuadora.setPosition(posB);
                                modo = Modo.B;
                            }
                        } else if (modo == Modo.A) {
                            if (objetivo.equals(slotX) && !objetivo.equals(slotB) ||
                                    objetivo.equals(slotX) && objetivo.equals(slotB)) {
                                Licuadora.setPosition(posX);
                                modo = Modo.X;
                            } else if (!objetivo.equals(slotX) && objetivo.equals(slotB)) {
                                Licuadora.setPosition(posB);
                                modo = Modo.B;
                            }
                        } else { // modo B
                            if (objetivo.equals(slotA) && !objetivo.equals(slotX) ||
                                    (objetivo.equals(slotA) && objetivo.equals(slotX))) {
                                Licuadora.setPosition(posA);
                                modo = Modo.A;
                            } else if (objetivo.equals(slotA) && objetivo.equals(slotX)) {
                                Licuadora.setPosition(posX);
                                modo = Modo.X;
                            }
                        }

                        estado_lanzar = 3;
                        estado_lanzador = true;
                        inicio_lanzador = System.nanoTime();


                    }
                }
            }
        } else {
            if (modo != Modo.X) {
                Licuadora.setPosition(posX);
                modo = Modo.X;
            }
        }
    }


    public boolean apriltag() {
        LLResult res = limaluz.getLatestResult();

        hayTag = false;
        fidCount = 0;
        stale = -1;


        if (res != null && res.isValid()) {
            stale = res.getStaleness();

            java.util.List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> tags =
                    res.getFiducialResults();

            fidCount = (tags == null) ? 0 : tags.size();
            hayTag = fidCount > 0;

            if (hayTag) {
                tx = res.getTx();
            }

            if (stale > 10) hayTag = false;
        }

        tieneObjetivo = hayTag;

        if (sensorM.isPressed()) {
            Lanzador_I.setPosition(0.5);
            Lanzador_D.setPosition(0.5);

            integral = 0.0;
            errorAnterior = 0.0;
            correccion = 0.5;

            telemetry.addLine("touch");
        } else {

            if (!tieneObjetivo) {
                // Stop
                Lanzador_I.setPosition(0.5);
                Lanzador_D.setPosition(0.5);

                // Reset controlador
                integral = 0.0;
                errorAnterior = 0.0;
                correccion = 0.5;
            } else {

                currentAngle = tx;

                double error = referencia - currentAngle;

                if (Math.abs(error) < 0.255) {
                    Lanzador_I.setPosition(0.5);
                    Lanzador_D.setPosition(0.5);
                    integral = 0.0;
                    correccion = 0.5;
                    return true;

                } else {

                    integral += error;
                    integral = Range.clip(integral, -30, 30);

                    double derivativa = error - errorAnterior;

                    double salida = Kp * error + Ki * integral + Kd * derivativa;

                    correccion = 0.5 + salida;
                    correccion = Range.clip(correccion, 0.0, 1.0);

                    if (correccion <= 0.01 || correccion >= 0.99) {
                        integral *= 0.7;
                    }

                    Lanzador_I.setPosition(correccion);
                    Lanzador_D.setPosition(correccion);
                }

                errorAnterior = error;
            }
        }
        return false;
    }


    public void statePathUpdate() {
        switch (estado) {
            case ESPERAR_ALINEACION:
                boolean alineado = apriltag();
                telemetry.addData("Estado", "Alineando antes de empezar...");

                if (alineado) {
                    lanzamiento(alineado);
                    if (estado_lanzar == 0 && !estado_lanzador) {
                        if (contador < 2) {
                            contador++;
                            licuadora_mov(contador);

                            
                        } else {
                            licuadora_mov(3);
                            // tiempo para que se mueva la licuadora
                            contador = 0;
                            follower.followPath(inicioARecoleccion1, true);
                            setPathState(INICIO_A_RECOLECCION);
                        }
                    }
                }
                break;


            case INICIO_A_RECOLECCION:
                follower.followPath(inicioARecoleccion1, true);
                setPathState(RECOLECCION);
                break;

            case RECOLECCION:
                M_lanzador2.setPower(1.0);
                recogedor_3();
                if (!"Nada".equals(slotX) && !"Nada".equals(slotA) && !"Nada".equals(slotB) && !follower.isBusy()) {
                    follower.followPath(recoleccion1, true);
                    setPathState(RECOLECCION1_A_LANZAMIENTO1);
                }
                break;

            case RECOLECCION1_A_LANZAMIENTO1:
                M_lanzador2.setPower(0.0);
                boolean var = apriltag();

                if (follower.getCurrentTValue() >= 0.2 && follower.getCurrentTValue() <= 0.5) {
                    licuadora_mov(0);
                    // tiempo
                }

                if (var && !follower.isBusy()) {
                    lanzamiento(var);
                    if (estado_lanzar == 0 && !estado_lanzador) {
                        if (contador < 2) {
                            contador++;
                            licuadora_mov(contador);
                            // tiempo para que se mueva la licuadora
                        } else {
                            licuadora_mov(3);
                            // tiempo para que se mueva la licuadora
                            contador = 0;
                            follower.followPath(recoleccion1Atiro1, true);
                            setPathState(LANZAMIENTO1_A_RECOLECCION2);
                        }
                    }

                }
                break;

            case LANZAMIENTO1_A_RECOLECCION2:
                if (!follower.isBusy()) {
                    follower.followPath(tiro1ARecoleccion2, true);
                    setPathState(RECOLECCION2);
                }
                M_lanzador2.setPower(1.0);
                break;

            case RECOLECCION2:

                recogedor_3();
                if (!"Nada".equals(slotX) && !"Nada".equals(slotA) && !"Nada".equals(slotB) && !follower.isBusy()) {
                    follower.followPath(recoleccion1, true);
                    setPathState(RECOLECCION1_A_LANZAMIENTO1);
                }
                break;

            case RECOLECCION2_A_LANZAMIENTO2:
                M_lanzador2.setPower(0.0);
                var = apriltag();

                if (follower.getCurrentTValue() >= 0.2 && follower.getCurrentTValue() <= 0.5) {
                    licuadora_mov(0); // idea que retorne true cuando ya se mueva
                }

                if (var && !follower.isBusy()) {
                    lanzamiento(var);
                    if (estado_lanzar == 0 && !estado_lanzador) {
                        if (contador < 2) {
                            contador++;
                            licuadora_mov(contador);
                            // tiempo para que se mueva la licuadora
                        } else {
                            licuadora_mov(3);
                            // tiempo para que se mueva la licuadora
                            contador = 0;
                            follower.followPath(recoleccion2Atiro2, true);
                            setPathState(LANZAMIENTO2_A_RECOLECCION3);
                        }
                    }

                }
                break;

            case LANZAMIENTO2_A_RECOLECCION3:
                if (!follower.isBusy()) {
                    follower.followPath(tiro2ARecoleccion3, true);
                    setPathState(RECOLECCION3);
                }
                break;

            case RECOLECCION3:
                M_lanzador2.setPower(1.0);
                recogedor_3();
                if (!"Nada".equals(slotX) && !"Nada".equals(slotA) && !"Nada".equals(slotB) && !follower.isBusy()) {
                    follower.followPath(recoleccion3, true);
                    setPathState(RECOLECCION3_A_LANZAMIENTO3);
                }
                break;

            case RECOLECCION3_A_LANZAMIENTO3:
                M_lanzador2.setPower(0.0);
                var = apriltag();

                if (follower.getCurrentTValue() >= 0.2 && follower.getCurrentTValue() <= 0.5) {
                    licuadora_mov(0);
                }

                if (var && !follower.isBusy()) {
                    lanzamiento(var);
                    if (estado_lanzar == 0 && !estado_lanzador) {
                        if (contador < 2) {
                            contador++;
                            licuadora_mov(contador);
                            // tiempo para que se mueva la licuadora
                        } else {
                            licuadora_mov(3);
                            // tiempo para que se mueva la licuadora
                            contador = 0;
                            follower.followPath(recoleccion3Atiro3, true);
                            setPathState(FIN);
                        }
                    }

                }
                break;


            case FIN:
                if (!follower.isBusy()) {
                    follower.followPath(fin, true);
                    setPathState(DONE);
                }
                break;

            case DONE:
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                }

                telemetry.addLine("DONE");
                telemetry.update();
                break;
            default:
                if (!follower.isBusy()) telemetry.addLine("New Path");
        }
    }

    public void setPathState ( int nuevoEstado){
        estado = nuevoEstado;
        if (pathTimer != null) pathTimer.resetTimer();
    }

    public void configurePinpoint () {

        pinpoint.setOffsets(-110 / 2.54, -160 / 2.54, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);


        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);


        //pinpoint.resetPosAndIMU();

    }

}

