#include <iostream>
#include <cmath>

float encoderMax = 360;
float xPos;
float yPos;
float head;

float distance(float value, float radius){ // takes in encoder ticks for "value" and radius of wheel
    return (float) (2 * (value / encoderMax) * M_PI * radius); // rotations
}

float heading(float hPrev, float left, float right, float b, float radius){
    float theta = (distance(left, radius) -
                   distance(right, radius)) / (b); // calculation for heading based on left and right wheel encoders
    float newHead = hPrev + theta;
    return newHead;
}

float positionChange(float prevPos, float left, float right, float radius){
    float newPos = (distance(left, radius) +
                    distance(right, radius)) / 2;
    return prevPos + newPos;
}

void initPos(){
    xPos = 0;
    yPos = 0;
    head = 0;
}