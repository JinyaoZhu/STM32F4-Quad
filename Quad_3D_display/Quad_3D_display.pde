import processing.serial.*;

int TEST = 0;

float [] angle = new float[3];
float [] position = new float[3];
Serial myPort;  // Create object from Serial class
byte[] inBuffer = new byte[512];

PShape BodyShape;
PShape QuadShape;
PShape CoordinateShape;



void setup()
{
  size(1280, 720, P3D);
  noStroke();
  BuildBody();
  println(Serial.list());

  if (TEST ==0) {
    String portName = Serial.list()[2];
    myPort = new Serial(this, portName, 115200);
  }
}
void draw()
{

  if (TEST == 0) {
    if (myPort.available()>0)
    {
      if (myPort.readBytesUntil('\n', inBuffer)>0)
      {
        String inputString = new String(inBuffer);
        String[] inputStringArr = split(inputString, ',');
        angle[0] = float(inputStringArr[0])*57.3;
        angle[2] = -float(inputStringArr[1])*57.3;
        angle[1] = float(inputStringArr[2])*57.3;
        position[0] = float(inputStringArr[3]);
        position[1] = float(inputStringArr[4]);
        position[2] = float(inputStringArr[5]);
        //println(angle[0]+","+angle[1]+","+angle[2]);
      }
    }
  }
  if (TEST != 0) {
    angle[0] = (1+angle[0])%360;
    angle[1] = 0;
    angle[2] = 0;
  }

  float body_x ;
  float body_y;
  float body_z;

  if (TEST == 1) {
    body_x = mouseX;
    body_y = mouseY;
    body_z = -50;
  } else
  {
    body_x = -position[0]*500+width/2;
    body_y = -position[2]*500+height;
    body_z = position[1]*500;
  }


  background(55);

  pushMatrix();
  translate(body_x, body_y, body_z);
  fill(color(250, 50, 50));
  sphere(10);
  popMatrix(); 

  fill(color(255, 255, 255));

  lights();
  pointLight(255, 255, 255, -200, 200, 200);

  pushMatrix();
  /* Body Frame */
  translate(body_x, body_y, body_z);
  scale(1, 1, 1);
  text("BodyFrame1", 0, 50, 0);
  text("X", 260, 0, 0);
  text("Y", 0, 0, -260);
  text("Z", 0, -260, 0);
  scale(5, 5, 5);
  shape(CoordinateShape);
  scale(0.2, 0.2, 0.2);
  rotateY(radians(angle[0])); /* yaw */
  rotateX(radians(angle[1])); /* pitch */
  rotateZ(radians(angle[2])); /* roll */

  text("BodyFrame2", 10, -50, 0);
  text("X", 260, 0, 0);
  text("Y", 0, 0, -260);
  text("Z", 0, -260, 0);
  scale(5, 5, 5);
  shape(BodyShape);
  popMatrix();

  /* Inertial Frame */
  pushMatrix();
  translate(width/2 - 450, height/2 + 250, 0);
  textSize(12);
  text("X", 260, 0, 0);
  textSize(20);
  text("Y", 0, 0, -260);
  textSize(12);
  text("Z", 0, -260, 0);
  text("InertialFrame", 0, 20, 0);
  scale(5, 5, 5);
  shape(CoordinateShape);
  popMatrix();

  textSize(12);
  text("Yaw  :" +angle[0]+"degree", 50, 90, 0);
  text("Pitch:" +angle[1]+"degree", 50, 120, 0);
  text("Roll :" +angle[2]+"degree", 50, 150, 0);
  
  text("pos x:" +position[0]+"m", 450, 90, 0);
  text("pos y:" +position[1]+"m", 450, 120, 0);
  text("pos z:" +position[2]+"m", 450, 150, 0);
}

float Y_MIN = -0.5;
float Y_MAX = 0.5;

void buildQuad()
{
  QuadShape = createShape();
  QuadShape.beginShape(QUADS);
  QuadShape.fill(color(80, 25, 250));     //upper

  QuadShape.vertex(2, Y_MIN, 0);
  QuadShape.vertex(26, Y_MIN, 24);
  QuadShape.vertex(24, Y_MIN, 26);
  QuadShape.vertex(0, Y_MIN, 2);

  QuadShape.vertex(0, Y_MIN, 2);
  QuadShape.vertex(-24, Y_MIN, 26);
  QuadShape.vertex(-26, Y_MIN, 24);
  QuadShape.vertex(-2, Y_MIN, 0);

  QuadShape.vertex(-2, Y_MIN, 0);
  QuadShape.vertex(-26, Y_MIN, -24);
  QuadShape.vertex(-24, Y_MIN, -26);
  QuadShape.vertex(0, Y_MIN, -2);

  QuadShape.vertex(0, Y_MIN, -2);
  QuadShape.vertex(24, Y_MIN, -26);
  QuadShape.vertex(26, Y_MIN, -24);
  QuadShape.vertex(2, Y_MIN, 0);

  QuadShape.vertex(2, Y_MIN, 0);
  QuadShape.vertex(0, Y_MIN, 2);
  QuadShape.vertex(-2, Y_MIN, 0);
  QuadShape.vertex(0, Y_MIN, -2);


  QuadShape.fill(color(235, 90, 0));      //down
  QuadShape.vertex(2, Y_MAX, 0);
  QuadShape.vertex(26, Y_MAX, 24);
  QuadShape.vertex(24, Y_MAX, 26);
  QuadShape.vertex(0, Y_MAX, 2);

  QuadShape.vertex(0, Y_MAX, 2);
  QuadShape.vertex(-24, Y_MAX, 26);
  QuadShape.vertex(-26, Y_MAX, 24);
  QuadShape.vertex(-2, Y_MAX, 0);

  QuadShape.vertex(-2, Y_MAX, 0);
  QuadShape.vertex(-26, Y_MAX, -24);
  QuadShape.vertex(-24, Y_MAX, -26);
  QuadShape.vertex(0, Y_MAX, -2);

  QuadShape.vertex(0, Y_MAX, -2);
  QuadShape.vertex(24, Y_MAX, -26);
  QuadShape.vertex(26, Y_MAX, -24);
  QuadShape.vertex(2, Y_MAX, 0);

  QuadShape.vertex(2, Y_MAX, 0);
  QuadShape.vertex(0, Y_MAX, 2);
  QuadShape.vertex(-2, Y_MAX, 0);
  QuadShape.vertex(0, Y_MAX, -2);

  QuadShape.fill(color(111, 250, 126));        //side
  QuadShape.vertex(2, Y_MIN, 0);
  QuadShape.vertex(2, Y_MAX, 0);
  QuadShape.vertex(26, Y_MAX, 24);
  QuadShape.vertex(26, Y_MIN, 24);


  QuadShape.vertex(26, Y_MIN, 24);
  QuadShape.vertex(26, Y_MAX, 24);
  QuadShape.vertex(24, Y_MAX, 26);
  QuadShape.vertex(24, Y_MIN, 26);

  QuadShape.vertex(0, Y_MIN, 2);
  QuadShape.vertex(0, Y_MAX, 2);
  QuadShape.vertex(24, Y_MAX, 26);
  QuadShape.vertex(24, Y_MIN, 26);

  QuadShape.vertex(0, Y_MIN, 2);
  QuadShape.vertex(0, Y_MAX, 2);
  QuadShape.vertex(-24, Y_MAX, 26);
  QuadShape.vertex(-24, Y_MIN, 26);

  QuadShape.vertex(-24, Y_MIN, 26);
  QuadShape.vertex(-24, Y_MAX, 26);
  QuadShape.vertex(-26, Y_MAX, 24);
  QuadShape.vertex(-26, Y_MIN, 24);

  QuadShape.vertex(-2, Y_MIN, 0);
  QuadShape.vertex(-2, Y_MAX, 0);
  QuadShape.vertex(-26, Y_MAX, 24);
  QuadShape.vertex(-26, Y_MIN, 24);

  QuadShape.vertex(-2, Y_MIN, 0);
  QuadShape.vertex(-2, Y_MAX, 0);
  QuadShape.vertex(-26, Y_MAX, -24);
  QuadShape.vertex(-26, Y_MIN, -24);

  QuadShape.vertex(-26, Y_MIN, -24);
  QuadShape.vertex(-26, Y_MAX, -24);
  QuadShape.vertex(-24, Y_MAX, -26);
  QuadShape.vertex(-24, Y_MIN, -26);

  QuadShape.vertex(-24, Y_MIN, -26);
  QuadShape.vertex(-24, Y_MAX, -26);
  QuadShape.vertex(0, Y_MAX, -2);
  QuadShape.vertex(0, Y_MIN, -2);

  QuadShape.vertex(0, Y_MIN, -2);
  QuadShape.vertex(0, Y_MAX, -2);
  QuadShape.vertex(24, Y_MAX, -26);
  QuadShape.vertex(24, Y_MIN, -26);

  QuadShape.vertex(24, Y_MIN, -26);
  QuadShape.vertex(24, Y_MAX, -26);
  QuadShape.vertex(26, Y_MAX, -24);
  QuadShape.vertex(26, Y_MIN, -24);

  QuadShape.vertex(26, Y_MIN, -24);
  QuadShape.vertex(26, Y_MAX, -24);
  QuadShape.vertex(2, Y_MAX, 0);
  QuadShape.vertex(2, Y_MIN, 0);

  QuadShape.endShape();
}


void buildCoordinate()
{
  CoordinateShape = createShape();

  CoordinateShape.beginShape(LINES);
  CoordinateShape.stroke(250);
  CoordinateShape.strokeWeight(1.5);
  /* X axis */
  CoordinateShape.vertex(0, 0, 0);
  CoordinateShape.vertex(50, 0, 0);
  CoordinateShape.vertex(50, 0, 0);
  CoordinateShape.vertex(48, 0, 1.2);
  CoordinateShape.vertex(50, 0, 0);
  CoordinateShape.vertex(48, 0, -1.2);

  /* Z axis */
  CoordinateShape.vertex(0, 0, 0);
  CoordinateShape.vertex(0, -50, 0);
  CoordinateShape.vertex(0, -50, 0);
  CoordinateShape.vertex(1.2, -48, 0);
  CoordinateShape.vertex(0, -50, 0);
  CoordinateShape.vertex(-1.2, -48, 0);

  /* Y axis */
  CoordinateShape.vertex(0, 0, 0);
  CoordinateShape.vertex(0, 0, -50);
  CoordinateShape.vertex(0, 0, -50);
  CoordinateShape.vertex(1.2, 0, -48);
  CoordinateShape.vertex(0, 0, -50);
  CoordinateShape.vertex(-1.2, 0, -48);

  CoordinateShape.endShape();
}

void BuildBody()
{
  buildQuad();
  buildCoordinate();
  BodyShape = createShape(GROUP);
  BodyShape.addChild(CoordinateShape);
  BodyShape.addChild(QuadShape);
}