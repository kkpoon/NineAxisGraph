import processing.serial.*;
import java.util.Arrays;
import java.util.Collections;
import java.util.ArrayList;
import java.text.DecimalFormat;

Serial serial;

final float ALPHA = 0.5f;
final double GYRO_SENS = 0.249337737;

byte[] buffer = new byte[1024];

int[] accel = new int[3];
int[] gyro = new int[3];
int[] mag = new int[3];
double[] g = new double[3];
double temperature = 0.0;
double pressure = 0.0;
double altitude = 0.0;
double heading = 0.0;

int time = 0;

ArrayList<Double>[] accHist = new ArrayList[3];

PShape compass;
PShape plane;

DecimalFormat df = new DecimalFormat("0.00");

void setup() {
    size(1200, 800, P3D);
    compass = createCompass(240);
    plane = createPlane();

    for (int i = 0; i < accHist.length; i++) {
      accHist[i] = new ArrayList<Double>(500);
      for (int j = 0; j < 500; j++) {
        accHist[i].add(new Double(0.0));
      }
    };

    println(Serial.list());
    serial = new Serial(this, Serial.list()[0], 38400);
    serial.write(65);
    
    time = millis();
}

void draw() {
    Arrays.fill(buffer, (byte)0);
    serial.readBytesUntil('\n', buffer);
    String in = new String(buffer);
    String[] data = in.trim().split(",");
    if (data.length != 12) return;
    try {
        accel[0] = Integer.parseInt(data[0]);
        accel[1] = Integer.parseInt(data[1]);
        accel[2] = Integer.parseInt(data[2]);
        gyro[0] = Integer.parseInt(data[3]);
        gyro[1] = Integer.parseInt(data[4]);
        gyro[2] = Integer.parseInt(data[5]);
        mag[0] = Integer.parseInt(data[6]);
        mag[1] = Integer.parseInt(data[7]);
        mag[2] = Integer.parseInt(data[8]);
        temperature = Double.parseDouble(data[9]);
        pressure = Double.parseDouble(data[10]);
        altitude = Double.parseDouble(data[11]);
        
        for (int i = 0; i < accel.length; i++) {
            // using low-pass filter
            g[i] = norm(accel[i], 0.0, 256.0) * ALPHA + g[i] * (1.0 - ALPHA);
            accHist[i].remove(0);
            accHist[i].add(new Double(g[i]));
        }
        
        heading = atan2(mag[1], mag[0]);
        if (heading < 0)
            heading += 2 * Math.PI;
        
        this.clear();
        colorMode(RGB, 255);
        background(255);
        lights();

        fill(0);
        printData();
        
        drawTriAxis(750, 200, 0);
        drawCompass(1000, 150, 0);
        drawHist(0, height/2, 0);
    } catch(Exception e) {
        println(e);
    }
    time = millis();
}

void printData() {
    text("Accelerometer Raw: ", 10, 30);
    text("x: " + accel[0], 200, 30);
    text("y: " + accel[1], 280, 30);
    text("z: " + accel[2], 360, 30);
    text("Gyro Raw: ", 10, 45);
    text("x: " + gyro[0], 200, 45);
    text("y: " + gyro[1], 280, 45);
    text("z: " + gyro[2], 360, 45);
    text("Mag Raw: ", 10, 60);
    text("mx: " + mag[0], 200, 60);
    text("my: " + mag[1], 280, 60);
    text("mz: " + mag[2], 360, 60);
    text("Heading: " + df.format(degrees((float)heading)), 480, 90);
    text("Temperature: " + df.format(temperature) + "C", 10, 90);
    text("Pressure: " + df.format(pressure / 100) + "hPa", 160, 90);
    text("Altitude: " + df.format(altitude), 320, 90);
    text("G Force: ", 10, 120);
    text("x: " + df.format(g[0]), 100, 120);
    text("y: " + df.format(g[1]), 180, 120);
    text("z: " + df.format(g[2]), 260, 120);
}

void drawHist(float x, float y, float z) {
    int histHeight = 50;
    
    pushMatrix();
    stroke(255, 0, 0);
    strokeWeight(1);
    translate(x, y, z);
    beginShape(LINES);
    vertex(5, 0, 0);
    vertex(accHist[0].size(), 0, 0);
    endShape();
    stroke(0, 0, 0);
    beginShape(LINES);
    for (int i = 0; i < accHist[0].size(); i++) {
        vertex(i+5, accHist[0].get(i).floatValue() * histHeight, 0);
    }
    endShape();
    popMatrix();
    
    pushMatrix();
    translate(x, y + 100, z);
    stroke(255, 0, 0);
    beginShape(LINES);
    vertex(5, 0, 0);
    vertex(accHist[1].size(), 0, 0);
    endShape();
    stroke(0, 0, 0);
    beginShape(LINES);
    for (int i = 0; i < accHist[1].size(); i++) {
        vertex(i+5, accHist[1].get(i).floatValue() * histHeight, 0);
    }
    endShape();
    popMatrix();
    
    pushMatrix();
    translate(x, y + 200, z);
    stroke(255, 0, 0);
    beginShape(LINES);
    vertex(5, 0, 0);
    vertex(accHist[2].size(), 0, 0);
    endShape();
    stroke(0, 0, 0);
    beginShape(LINES);
    for (int i = 0; i < accHist[2].size(); i++) {
        vertex(i+5, accHist[2].get(i).floatValue() * histHeight, 0);
    }
    endShape();
    popMatrix();
}

void drawTriAxis(float x, float y, float z) {
    int length = 120;
    pushMatrix();
    translate(x, y, z);
    noFill();
    
    stroke(204);
    strokeWeight(4);
    PShape triPlane = createPlane();
    triPlane.rotateX(PI/2);
    triPlane.rotateZ(-PI/2);
    
    triPlane.rotateX(-atan2((float)g[0], (float)g[2]));
    triPlane.rotateY(atan2((float)g[1], (float)g[2]));

//  TODO sensor fusion (acc + gyro)    
//    int currentTime = millis();
//    double xRot = radians(gyro[0]) / GYRO_SENS * (currentTime - time) / 1000;
//    double yRot = radians(gyro[1]) / GYRO_SENS * (currentTime - time) / 1000;
//    double zRot = radians(gyro[2]) / GYRO_SENS * (currentTime - time) / 1000;
//    
//    triPlane.rotateX((float)yRot);
//    triPlane.rotateY((float)zRot);
//    triPlane.rotateZ((float)xRot);
//    
    
    strokeWeight(2);
    stroke(0,0,255);
    PShape xAxis = createAxis((float)g[0] * length);
    xAxis.rotateX(HALF_PI);
    stroke(255,0,0);
    PShape yAxis = createAxis((float)g[1] * length);
    yAxis.rotateY(HALF_PI);
    stroke(0,255,0);
    PShape zAxis = createAxis((float)g[2] * -length);
    zAxis.rotateZ(HALF_PI);

    PShape triAxis = createShape(GROUP);
    triAxis.addChild(triPlane);    
    triAxis.addChild(xAxis);
    triAxis.addChild(yAxis);
    triAxis.addChild(zAxis);
    triAxis.rotateX(-PI/4);
    triAxis.rotateY(-PI/4);
    //triAxis.rotateZ(PI/4);
    shape(triAxis, 0, 0);
    popMatrix();
    
    // text label
    pushMatrix();
    translate(x, y, z);
    rotateX(-PI/4);
    rotateY(-PI/4);
    fill(0);
    stroke(0);
    textAlign(CENTER, CENTER);
    text("x", (float)g[0] * (length + 5),0,0);
    text("y", 0,0,-(float)g[1] * (length + 5));
    text("z", 0,-(float)g[2] * (length + 5),0);
    popMatrix();
}

void drawCompass(float x, float y, float z) {
    float rotation = (float)heading * -1.0;
    pushMatrix();
    translate(x, y, z);
    rotate(rotation);
    shape(compass, 0, 0);
    popMatrix();
    
    for (int i = 0; i < 360; i+=30) {
        pushMatrix();
        translate(x, y, z);
        rotate(radians(i));
        rotate(rotation);
        stroke(0);
        strokeWeight(1);
        fill(0);
        textAlign(CENTER, CENTER);
        text(i, 0, -110, 0);
        popMatrix();
    }
    
    pushMatrix();
    translate(x, y, z);
    textAlign(LEFT, CENTER);
    strokeWeight(2);
    fill(0);
    shape(plane, 0, 0);
    text(degrees((float)heading), -30,-60,0);
    popMatrix();
}

PShape createAxis(float length) {
    PShape axis = createShape();
    axis.beginShape(LINES);
    axis.vertex(0,0,0);
    axis.vertex(length,0,0);
    axis.endShape();
    return axis;
}

PShape createCompass(int width) {
    PShape compass = createShape(GROUP);
    PShape circle = createShape(ELLIPSE, -width/2, -width/2, width, width);
    compass.addChild(circle);
    int inner = width/2 - 30;
    int outer = width/2 - 20;
    for (int i = 0; i < 360; i+=10) {
        compass.addChild(createCompassTick(i, inner, outer));
    }
    return compass;
}

PShape createCompassTick(int degree, int innerRadius, int outerRadius) {
    PShape tick = createShape();
    tick.beginShape(LINES);
    tick.vertex(0, -innerRadius, 0);
    tick.vertex(0, -outerRadius, 0);
    tick.endShape();
    tick.rotate(radians(degree));
    return tick;
}

PShape createPlane() {
    PShape plane = createShape();
    plane.beginShape(LINES);
    plane.vertex(0,-50,0);
    plane.vertex(0,50,0);
    plane.endShape();
    plane.beginShape(LINES);
    plane.vertex(-40, -20, 0);
    plane.vertex(40, -20, 0);
    plane.endShape();
    plane.beginShape(LINES);
    plane.vertex(-20, 50, 0);
    plane.vertex(20, 50, 0);
    plane.endShape();
    return plane;
}

