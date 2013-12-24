import processing.serial.*;
import java.util.Arrays;
import java.text.DecimalFormat;

Serial serial;

byte[] buffer = new byte[1024];

int[] accel = new int[3];
int[] gyro = new int[3];
int[] mag = new int[3];
double[] g = new double[3];
double temperature = 0.0;
double pressure = 0.0;
double altitude = 0.0;
double heading = 0.0;

DecimalFormat df = new DecimalFormat("0.00");

void setup() {
    size(1200, 800, P3D);

    println(Serial.list());
    serial = new Serial(this, Serial.list()[2], 38400);
    serial.write(65);
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
            g[i] = norm(accel[i], 0.0, 256.0);
        }
        
        heading = atan2(mag[1], mag[0]);
        if (heading < 0)
            heading += 2 * Math.PI;
        
        this.clear();
        background(255);
        fill(0);
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
        text("G Force: ", 500, 30);
        text("x: " + df.format(g[0]), 700, 30);
        text("y: " + df.format(g[1]), 780, 30);
        text("z: " + df.format(g[2]), 860, 30);
        
        colorMode(RGB, 255);
        
        fill(204);
        lights();
        pushMatrix();
        translate(width/2, height/2, 0);
        beginShape(LINES);
        strokeWeight(2);
        noFill();
        
        // x-axis
        stroke(0,0,255);
        vertex(0,0,0);
        vertex((float)g[0] * 200,0,0);
        
        // z-axis
        stroke(0,255,0);
        vertex(0,0,0);
        vertex(0,(float)g[2] * -200,0);
        
        // y-axis
        stroke(255,0,0);
        vertex(0,0,0);
        vertex(0,0,(float)g[1] * 200);
        
        rotateX(radians(10));
        rotateY(radians(10));
        //rotateZ(radians(45));
        endShape();
        popMatrix();
        
        textAlign(LEFT, CENTER);
        pushMatrix();
        translate(width/2, height/2, 0);
        fill(0);
        stroke(0);
        text("x", (float)g[0] * 200,0,0);
        text("y", 0,0,(float)g[1] * 200);
        text("z", 0,(float)g[2] * -200,0);
        rotateX(radians(20));
        rotateY(radians(20));
        rotateZ(radians(20));
        popMatrix();
        
        pushMatrix();
        translate(width/4, height/2, 0);
        noFill();
        stroke(0);
        strokeWeight(1);
        ellipse(0,0,150,150);
        beginShape(LINES);
        for (int i = 0; i < 360; i+=10) {
            vertex(cos(radians(i - 90)) * 100, sin(radians(i - 90)) * 100, 0);
            vertex(cos(radians(i - 90)) * 90, sin(radians(i - 90)) * 90,0);
            
        }
        rotate((float)heading * -1.0);
        endShape();
        textAlign(CENTER, CENTER);
        for (int i = 0; i < 360; i+=30) {
            text(i, cos(radians(i - 90)) * 120, sin(radians(i - 90)) * 120,0);
        }
        rotate((float)heading * -1.0);
        popMatrix();
        
        pushMatrix();
        translate(width/4, height/2, 0);
        textAlign(LEFT, CENTER);
        strokeWeight(2);
        fill(0);
        beginShape(LINES);
        vertex(0,-50,0);
        vertex(0,50,0);
        endShape();
        text(degrees((float)heading), -30,-60,0);
        popMatrix();
    } catch(Exception e) {
        println(e);
    }
}
