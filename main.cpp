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

class Box {
public:
    float v[4][2]; // The 4 corners.
    
    float pivot_x, pivot_y; 
    float w, h;
    int colour;

    Box(int box_width = 20, int box_height = 40, int x = 160, int y = 120, int c = ILI9341_RED) {
        colour = c;
        w = box_width;
        h = box_height;
        
        // Pivot is now the centre (x, y)
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

    // Calculates rotation around the center (pivot).
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
      float dx = X - close_point[0];
      float dy = Y - close_point[1];
      float dist_sq = (dx*dx) + (dy*dy);
      // Edge collision:
      // Adding a small error term for better detection.
      if (dist_sq <= (size + 0.1)*(size + 0.1)){
         float length = sqrt(dist_sq);

         // Prevents division by zero to avoid errors.
         if (length == 0) return;

         // Normal direction:
         float n_x = dx/length;
         float n_y = dy/length;
         float v_x = speed[0];
         float v_y = speed[1];

         // Position correction to avoid getting stuck inside the box.
         float overlap = (size + 0.01) - length;

         X += n_x*overlap;
         Y += n_y*overlap;

         // Velocity along normal direction:
         float dot_product = v_x*n_x + v_y*n_y;

         // Velcro effect check (Only reflect if moving towards the edge).
         if (dot_product < 0) {
            /* Updating the components of the ball's velocity 
            based on the laws of reflection.*/
                speed[0] = v_x - 2 * dot_product * n_x;
                speed[1] = v_y - 2 * dot_product * n_y;
            }
      }
   }

};

void ball_ball_collision(Ball &ball1, Ball &ball2){
     //Checking for ball to ball collision.
   float del_x = ball1.X - ball2.X;
   float del_y = ball1.Y - ball2.Y;
   // Adding a small error term for better detection.
   float closest_dist = ball1.size + ball2.size + 0.1;
   if (del_x*del_x + del_y*del_y <= closest_dist*closest_dist){
      float length = sqrt(del_x*del_x + del_y*del_y);
      // Finding the normal and tangential direction.
      float n_x = del_x/length;
      float n_y  = del_y/length;
      float t_x = -n_y;
      float t_y = n_x;

      // Position correction to avoid the balls sticking together.
      float overlap = (closest_dist) - length;

      // Pushing the balls along the normal direction to make it not overlap.
      ball1.X += overlap*0.5*n_x;
      ball1.Y += overlap*0.5*n_y;

      ball2.X -= overlap*0.5*n_x;
      ball2.Y -= overlap*0.5*n_y;

      float v1x = ball1.speed[0];
      float v1y = ball1.speed[1];
      float v2x = ball2.speed[0];
      float v2y = ball2.speed[1];

      // Finding the velocities along the normal and tangential direction.
      float v1_n_old = n_x*v1x + n_y*v1y;
      float v2_n_old = n_x*v2x + n_y*v2y;
      float v1_t = t_x*v1x + t_y*v1y;
      float v2_t = t_x*v2x + t_y*v2y;

      float m1 = ball1.mass;
      float m2 = ball2.mass;

      // Update the velocity only if they are moving towards each other.
      if (v1_n_old - v2_n_old < 0) {

      // Calculating the new velocity along the normal direction.
      float v1_n = (v1_n_old * (m1 - m2) + 2 * m2 * v2_n_old) / (m1 + m2);
      float v2_n = (v2_n_old * (m2 - m1) + 2 * m1 * v1_n_old) / (m1 + m2);

      // Calculating the new velocities for the balls.
      ball1.speed[0] = v1_n*n_x + v1_t*t_x;
      ball1.speed[1] = v1_n*n_y + v1_t*t_y;
      ball2.speed[0] = v2_n*n_x + v2_t*t_x;
      ball2.speed[1] = v2_n*n_y + v2_t*t_y;
   }}
}



Ball ball1(10, 10, 5, 5, 5,10,ILI9341_GREEN);
Ball ball2(310, 10, 3, 5, -5,10, ILI9341_ORANGE);
Box box(40, 60);

void setup(){
   tft.begin();
   tft.setRotation(3); 
   tft.fillScreen(ILI9341_BLACK);
   box.rotate_box(360);
   box.draw_box();
   ball1.draw();
   ball2.draw();
}

void loop(){
   // Clearing the objects from the old frame.
   ball1.erase();
   ball2.erase();

   // Updating the coordinates of the ball for each frame.
   ball1.update();
   ball2.update();

   // Checking for wall collisions.
   ball1.wall_collision();
   ball2.wall_collision();

   // Checking for ball-ball collisions.
   ball_ball_collision(ball1, ball2);

   // Left edge.
   ball1.box_edge_collisions(box.v[0][0], box.v[0][1], box.v[3][0], box.v[3][1]);
   ball2.box_edge_collisions(box.v[0][0], box.v[0][1], box.v[3][0], box.v[3][1]);
    // Right edge.
   ball1.box_edge_collisions(box.v[2][0], box.v[2][1], box.v[1][0], box.v[1][1]);
   ball2.box_edge_collisions(box.v[2][0], box.v[2][1], box.v[1][0], box.v[1][1]);
    // Top edge.
   ball1.box_edge_collisions(box.v[1][0], box.v[1][1], box.v[0][0], box.v[0][1]);
   ball2.box_edge_collisions(box.v[1][0], box.v[1][1], box.v[0][0], box.v[0][1]);
    // Bottom edge.
   ball1.box_edge_collisions(box.v[3][0], box.v[3][1], box.v[2][0], box.v[2][1]);
   ball2.box_edge_collisions(box.v[3][0], box.v[3][1], box.v[2][0], box.v[2][1]);

    // Displaying the objects at the new positions.
   ball1.draw();
   ball2.draw();
   box.draw_box();

   delay(25);
}
