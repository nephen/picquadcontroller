import processing.serial.*;
Serial port;  // The serial port

void setup() {
  size(500, 500, P3D);
  
  
  port = new Serial(this,  "COM18",19200 );  //change to your port settings  115200 / 19200
  

  noStroke();
  fill(204, 204);
  textMode(SCREEN);
  ortho(-width/2, width/2, -height/2, height/2, -10, 10); 
}

float viewAngle[] = {25,-35};

final static int SKIP_FRAMES = 10;    //skip frames on slow computers
int skip_counter = 0;

int view = 1;
String[] viewStr = {"Global & Body Frame","Body Frame in Global Coordinates", "Global Frame in Body's Coordinates"};
void keyPressed() {
  if (key == 'v' || key == 'V') view = (view+1) % 3;
}

void draw() {
  background(#EEFFFF);
  lights();  
  fill(0);


  
  //click and drag to change view angle
  if (mousePressed == true){
    viewAngle[0] =  (mouseX - width/2 );
    viewAngle[1] = -(mouseY - height/2) ;
  }   

  //get DCM matrix from serial input
  String s = port.readStringUntil(10);
  

  if(s != null){  
    float a[] = parse_numbers(s);
    if(a.length >= 4 + 3*3)
      for(int i=0;i<3;i++){
        acc[i] = a[1+i];
        for(int j=0;j<3;j++)
          dcm[i][j] = a[4 + i*3 + j];
      }          
  } 

  
  text("Camera View Angle(click on center and drag): " + viewAngle[0] + " , " + viewAngle[1]+ "\n"+"View:"+viewStr[view]+" (press V to change)", 10,20);  


  
  translate( width/2, height/2, 0);
 
  for(int i=0; i<100;i++){
    rotateX(viewAngle[1] / 100 * PI/180);    
    rotateY(viewAngle[0] / 100 * PI/180);
  }    
 
 
  scale(100);  
 
  draw_dcm();


    
}

//split a list of comma or space separated numbers
float[]  parse_numbers(String s){
  String p[] = split(s,',');
  float f[] = new float[p.length];
  for(int i=0;i<p.length;i++)
    f[i] = parseFloat(p[i]);
  return f;    
}


float dcm[][] =  {{1,0,0},{0,1,0},{0,0,1}};
float acc[] = {0,0,1};

void draw_dcm(){
  float d = 0.05;
  //draw axis
  strokeWeight(1);
  stroke(0x330000FF);    // X - blue (screen's -X axis)
  line(-2,0,0, 2,0,0);  
  fill(0x330000FF);  
  beginShape(TRIANGLES);
  vertex(-2.1,0,0);vertex(-2,d,d);vertex(-2,d,-d);
  vertex(-2.1,0,0);vertex(-2,d,-d);vertex(-2,-d,-d);  
  vertex(-2.1,0,0);vertex(-2,-d,-d);vertex(-2,-d,d);    
  vertex(-2.1,0,0);vertex(-2,-d,d);vertex(-2,d,d);      
  endShape();
  stroke(0x33FF0000);    // Y - red (screen's Z axis)
  line(-0,0,-2, 0,0,2);  
  fill(0x33FF0000);  
  beginShape(TRIANGLES);
  vertex(0,0,2.1);vertex(d,d,2);vertex(d,-d,2);
  vertex(0,0,2.1);vertex(d,-d,2);vertex(-d,-d,2);  
  vertex(0,0,2.1);vertex(-d,-d,2);vertex(-d,d,2);    
  vertex(0,0,2.1);vertex(-d,d,2);vertex(d,d,2);      
  endShape();  
  stroke(0x3300FF00);    // Z - green (screen's -Y axis)
  line(-0,-2,0, 0,2,0);  
  fill(0x3300FF00);  
  beginShape(TRIANGLES);
  vertex(0,-2.1,0);vertex(d,-2,d);vertex(d,-2,-d);
  vertex(0,-2.1,0);vertex(d,-2,-d);vertex(-d,-2,-d);  
  vertex(0,-2.1,0);vertex(-d,-2,-d);vertex(-d,-2,d);    
  vertex(0,-2.1,0);vertex(-d,-2,d);vertex(d,-2,d);      
  endShape();

  
  //show inverse gravitation vector in Device coordinates
  if(0==view || 2==view){
    strokeWeight(5); 
    stroke(#000000);   
    draw_vector(acc[0],acc[1],acc[2]);

  
    //show World in Device coordinates
    strokeWeight(3); 
    stroke(#0000FF);  
    draw_vector(dcm[0][0],dcm[0][1],dcm[0][2]);
    stroke(#FF0000);    
    draw_vector(dcm[1][0],dcm[1][1],dcm[1][2]);
    stroke(#00FF00);    
    draw_vector(dcm[2][0],dcm[2][1],dcm[2][2]);
  }
  
  if(0==view || 1==view){ 
    //show Device in Word coordinates (translated DCM matrix)
    strokeWeight(6); 
    stroke(0==view ? 0x330000FF : #0000FF);      // X - blue 
    draw_vector(dcm[0][0],dcm[1][0],dcm[2][0]);
      stroke(0==view ? 0x33FF0000 : #FF0000);    // Y - red
    draw_vector(dcm[0][1],dcm[1][1],dcm[2][1]);
    stroke(0==view ? 0x3300FF00 : #00FF00);      // Z - green   
    draw_vector(dcm[0][2],dcm[1][2],dcm[2][2]);
    
  }

}; 

void draw_vector(float x,float y,float z){
  //Convert from DCM to screen coordinate system:
  //DCM(X) = - Screen(X)  
  //DCM(Y) =   Screen(Z)
  //DCM(Z) = - Screen(Y)
  line(0,0,0,-x,-z,y);     
}



