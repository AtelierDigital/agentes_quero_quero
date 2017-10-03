// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

//  [ADi] Alem do flock(), temos um comportamento quero_quero()

float swt = 25.0;     //sep.mult(25.0f);
float awt = 4.0;      //ali.mult(4.0f);
float cwt = 5.0;      //coh.mult(5.0f);
float maxspeed = 1;
float maxforce = 0.025;

//  [ADi]
float chaseWt = 10.0;  //  chases a target
//float avoidFloorWt = 30.f;  //  avoids floor
PVector rightStop, leftStop;



// Flocking
// Daniel Shiffman <http://www.shiffman.net>
// The Nature of Code, Spring 2009

// Boid class
// Methods for Separation, Cohesion, Alignment added

class Boid {

  PVector pos;
  PVector vel;
  PVector acc;
  float r;

  //  [ADi]
  float rEnterFlee_sq, rExitFlee_sq;
  static final int stateChase = 0;
  static final int stateFlee  = 1;
  int state;
  PVector target;
  float yThreshold;

  Boid(float x, float y) {
    acc = new PVector(0,0);
    vel = new PVector(random(-1,1),random(-1,1));
    pos = new PVector(x,y);
    r = 2.0;
    
    //  [ADi]
    state = stateChase;
    rEnterFlee_sq = 120*120;
    rExitFlee_sq  = 500*500;
    target = new PVector();
    yThreshold = 0.6*height; 
    leftStop = new PVector(width/10.0, height/10.0);
    rightStop = new PVector(9.0*width/10.0, height/10.0);
  }

  void run(ArrayList<Boid> boids) {
    flock(boids);
    quero_quero();  //  ADi
    update();
    borders();
    render();
  }

  void applyForce(PVector force) {
    // We could add mass here if we want A = F / M
    acc.add(force);
  }

  // We accumulate a new acceleration each time based on three rules
  void flock(ArrayList<Boid> boids) {
    PVector sep = separate(boids);   // Separation
    PVector ali = align(boids);      // Alignment
    PVector coh = cohesion(boids);   // Cohesion
    // Arbitrarily weight these forces
    sep.mult(swt);
    ali.mult(awt);
    coh.mult(cwt);
    // Add the force vectors to acceleration
    applyForce(sep);
    applyForce(ali);
    applyForce(coh);
  }
  
  //
  //  [ADi]
  void quero_quero()
  {   
    target.x = mouseX;
    target.y = mouseY;
    
    float sqrDist = sq(pos.x-target.x)+sq(pos.y-target.y);
    //  Inside enter flee area
    if( sqrDist < rEnterFlee_sq )
    {
      state = stateFlee;
      //applyForce( new PVector(0.0,-10.0*maxforce).mult(chaseWt) );
      applyForce( seek(target).mult(-chaseWt) );
    }
    else if(sqrDist < rEnterFlee_sq)
    {
      //  Minor than exit flee distance, but still in flee state
      applyForce( seek(target).mult(-chaseWt) );
    }
    else if(state == stateFlee){  //  Great than exit flee distance --> to a stop point...
      if(vel.x < 0)  {
        float sqrDist2 = sq(pos.x-leftStop.x)+sq(pos.y-leftStop.y);
        applyForce( seek(leftStop).mult(chaseWt) );
        if(sqrDist2 < 40*40) {
          state = stateChase;
        }
      }
      else {
        applyForce( seek(rightStop).mult(chaseWt) );
        
        float sqrDist2 = sq(pos.x-rightStop.x)+sq(pos.y-rightStop.y);
        applyForce( seek(rightStop).mult(chaseWt) );
        if(sqrDist2 < 40*40) {
          state = stateChase;
        }
      } 
    }
    else {  //  ... or return to seek 
      state = stateChase;
      applyForce( seek(target).mult(chaseWt) );
    }
    
    //  Always avoids ground
    if(pos.y > yThreshold) {
      state = stateFlee;
      //applyForce( new PVector(0.0, -0.1).mult(pos.y-yThreshold) );
    }
  }

  // Method to update position
  void update() {
    // Update velocity
    vel.add(acc);
    // Limit speed
    vel.limit(maxspeed);
    pos.add(vel);
    // Reset accelertion to 0 each cycle
    acc.mult(0);
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target,pos);  // A vector pointing from the position to the target

    // Normalize desired and scale to maximum speed
    desired.normalize();
    desired.mult(maxspeed);
    // Steering = Desired minus Velocity
    PVector steer = PVector.sub(desired,vel);
    steer.limit(maxforce);  // Limit to maximum steering force

    return steer;
  }

  void render() {
    // Draw a triangle rotated in the direction of velocity
    float theta = vel.heading2D() + radians(90);
    fill(175);
    stroke(0);
    pushMatrix();
    translate(pos.x,pos.y);
    rotate(theta);
    beginShape(TRIANGLES);
    vertex(0, -r*2);
    vertex(-r, r*2);
    vertex(r, r*2);
    endShape();
    popMatrix();
  }

  // Wraparound
  void borders() {
    if (pos.x < -r) pos.x = width+r;
    if (pos.y < -r) pos.y = height+r;
    if (pos.x > width+r) pos.x = -r;
    if (pos.y > height+r) pos.y = -r;
  }

  // Separation
  // Method checks for nearby boids and steers away
  PVector separate (ArrayList<Boid> boids) {
    float desiredseparation = 25.0;
    PVector steer = new PVector(0,0);
    int count = 0;
    // For every boid in the system, check if it's too close
    for (Boid other : boids) {
      float d = PVector.dist(pos,other.pos);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)) {
        // Calculate vector pointing away from neighbor
        PVector diff = PVector.sub(pos,other.pos);
        diff.normalize();
        diff.div(d);        // Weight by distance
        steer.add(diff);
        count++;            // Keep track of how many
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.div((float)count);
      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(maxspeed);
      steer.sub(vel);
      steer.limit(maxforce);
    }
    return steer;
  }

  // Alignment
  // For every nearby boid in the system, calculate the average velocity
  PVector align (ArrayList<Boid> boids) {
    float neighbordist = 50.0;
    PVector steer = new PVector();
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(pos,other.pos);
      if ((d > 0) && (d < neighbordist)) {
        steer.add(other.vel);
        count++;
      }
    }
    if (count > 0) {
      steer.div((float)count);
      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(maxspeed);
      steer.sub(vel);
      steer.limit(maxforce);
    }
    return steer;
  }

  // Cohesion
  // For the average position (i.e. center) of all nearby boids, calculate steering vector towards that position
  PVector cohesion (ArrayList<Boid> boids) {
    float neighbordist = 50.0;
    PVector sum = new PVector(0,0);   // Start with empty vector to accumulate all positions
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(pos,other.pos);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.pos); // Add position
        count++;
      }
    }
    if (count > 0) {
      sum.div((float)count);
      return seek(sum);  // Steer towards the position
    }
    return sum;
  }
}