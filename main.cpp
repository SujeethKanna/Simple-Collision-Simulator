#include <Arduino.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_GFX.h>
#include <SPI.h>

#define DC 4
#define CS 5

const int screen_height = 240;
const int screen_width = 320;

Adafruit_ILI9341 tft = Adafruit_ILI9341(CS, DC);

std::array<float, 2> closest_point(float x, float y, float x1, float y1, float x2, float y2){
   float d_X = x2 - x1;
   float d_Y = y2 - y1;
   float P_B_x = x - x1;
   float P_B_y = y - y1;
   float t = (P_B_x*d_X + P_B_y*d_Y)/(d_X*d_X + d_Y*d_Y);
   if (t < 0) t = 0;
   else if (t > 1) t = 1;
   float closest_x = x1 + t*(d_X);
   float closest_y = y1 + t*(d_Y);
   std::array<float, 2> closest_point = {closest_x, closest_y};
   return closest_point;
}

float dist(float x1, float y1, float x2, float y2){
   float d_x = x1 - x2;
   float d_y = y1 - y2;
   float distance = sqrt(d_x*d_x + d_y*d_y);
   return distance;
}

class Box {
public:
    float v[4][2]; // The 4 corners: TL, TR, BR, BL
    
    float pivot_x, pivot_y; // Center of the box
    float w, h;
    int colour;

    Box(int box_width = 20, int box_height = 40, int x = 160, int y = 120, int c = ILI9341_RED) {
        colour = c;
        w = box_width;
        h = box_height;
        
        // Pivot is now the CENTER (x, y)
        pivot_x = x;
        pivot_y = y; 

        rotate_box(0); 
    }

    void draw_box() {
        for (int i = 0; i < 4; i++) {
            int next = (i + 1) % 4; 
            tft.drawLine((int)v[i][0], (int)v[i][1], 
                         (int)v[next][0], (int)v[next][1], colour);
        }
    }

    void clear_box() {
        for (int i = 0; i < 4; i++) {
            int next = (i + 1) % 4; 
            tft.drawLine((int)v[i][0], (int)v[i][1], 
                         (int)v[next][0], (int)v[next][1], ILI9341_BLACK);
        }
    }

    // Calculates absolute rotation around the center.
    void rotate_box(float angle) {
        float c = cos(angle*PI/180);
        float s = sin(angle*PI/180);
        
        float hw = w / 2.0; // Half-width
        float hh = h / 2.0; // Half-height

        // 0: Top-Left (-hw, -hh)
        v[0][0] = pivot_x + (-hw * c - -hh * s);
        v[0][1] = pivot_y + (-hw * s + -hh * c);

        // 1: Top-Right (+hw, -hh)
        v[1][0] = pivot_x + (hw * c - -hh * s);
        v[1][1] = pivot_y + (hw * s + -hh * c);

        // 2: Bottom-Right (+hw, +hh)
        v[2][0] = pivot_x + (hw * c - hh * s);
        v[2][1] = pivot_y + (hw * s + hh * c);

        // 3: Bottom-Left (-hw, +hh)
        v[3][0] = pivot_x + (-hw * c - hh * s);
        v[3][1] = pivot_y + (-hw * s + hh * c);
    }
};

class Ball{
   public:
   float X, Y , mass;
   int size, colour;
   float speed[2];

   Ball(int x, int y, int m,int sp_x, int sp_y, int s, int c){
      X = x;
      Y = y;
      mass = m;
      size = s;
      colour = c;
      speed[0] = sp_x;
      speed[1] = sp_y;
   }

   void draw(){
      tft.fillCircle((int)X, (int)Y, size, colour);
   }
   void erase(){
      tft.fillCircle((int)X, (int)Y, size, ILI9341_BLACK);
   }
   void update(){
      X += speed[0];
      Y += speed[1];  
   }
   void wall_collision(){
      // Forcing the ball's position out of the wall to avoid Velcro effect.
      if (X <= 10) {
         speed[0] = -speed[0];
         X = 11; 
      }
      else if (X > screen_width - size) {
         speed[0] = -speed[0];
         X = screen_width - size - 1; 
      }
      if (Y <= 10) {
         speed[1] = -speed[1];
         Y = 11; 
      }
      else if (Y >= screen_height - size) {
         speed[1] = -speed[1];
         Y = screen_height - size - 1;
      }
      }

   void box_edge_collisions(float x1, float y1, float x2, float y2){
      // Computes the closest point.
      std::array<float, 2> close_point = closest_point(X, Y, x1, y1, x2, y2);
      // Edge collision:
      if (dist(close_point[0], close_point[1], X, Y) <= size + 0.0001 ){
         float length = dist(close_point[0], close_point[1], X, Y);
         // Normal direction:
         float n_x = (X - close_point[0])/length;
         float n_y = (Y - close_point[1])/length;
         float v_x = speed[0];
         float v_y = speed[1];

         // Velocity along normal direction:
         float dot_product = v_x*n_x + v_y*n_y;

         /* Making sure that the ball is moving towards 
         the edge when colliding to avoid velcro effect.*/ 
         if (dot_product < 0) {
            /* Updating the components of the ball's velocity 
            based on the laws of reflection.*/
                speed[0] = v_x - 2 * dot_product * n_x;
                speed[1] = v_y - 2 * dot_product * n_y;
            }
      }
   }

};

Ball ball(10, 10, 5, 5, 5,10,ILI9341_GREEN);
Box box(40, 60);

void setup(){
   tft.begin();
   tft.setRotation(3); 
   tft.fillScreen(ILI9341_BLACK);
   box.rotate_box(100);
   box.draw_box();
   ball.draw();
}

void loop(){
   // Clearing the objects from the old frame.
   ball.erase();

   // Updating the coordinates of the ball for each frame.
   ball.update();

   // Checking for wall collisions.
   ball.wall_collision();

   // Left edge.
   ball.box_edge_collisions(box.v[0][0], box.v[0][1], box.v[3][0], box.v[3][1]);
    
    // Right edge.
   ball.box_edge_collisions(box.v[2][0], box.v[2][1], box.v[1][0], box.v[1][1]);
    
    // Top edge.
   ball.box_edge_collisions(box.v[1][0], box.v[1][1], box.v[0][0], box.v[0][1]);
    
    // Bottom edge.
   ball.box_edge_collisions(box.v[3][0], box.v[3][1], box.v[2][0], box.v[2][1]);

    // Displaying the objects at the new positions.
   ball.draw();
   box.draw_box();

   delay(25);
}
