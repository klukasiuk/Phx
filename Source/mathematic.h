#pragma once

#include <cmath>

const float PI = 3.14159265358979f;

// Macierz Rotacji
 class RotMat
 {
 public:

  float sinus;
  float cosinus;

  RotMat(float angle)
  {
    angle = angle/360 * 2 * PI;      // Stopnie na radiany

    cosinus = cos(angle);  
    sinus = sin(angle);
  }

 };

// Wektor dwuwymiarowy
 class Vec2
{
 public:

 float x;   
 float y;


 Vec2()                   
 { 
	 x=0.0f;
	 y=0.0f;
 }   

 Vec2(float X, float Y)
 {
	 x=X;
	 y=Y;
 }   

 Vec2 operator +(Vec2 v)
 {
  return Vec2( this->x + v.x, this->y + v.y );
 }

 Vec2 operator -(Vec2 v)
 {
  return Vec2( this->x - v.x, this->y - v.y );
 }

 Vec2 operator *(float v) 
 {
  return Vec2( this->x * v , this->y * v );
 }

 Vec2 operator /(float v)
 {
  return Vec2( this->x / v , this->y / v );
 }

 bool operator ==(Vec2 v)
{
    if( ( this->x == v.x ) &&( this->y == v.y ) )
         return true;
    else
         return false;
}

 bool operator !=(Vec2 v)
{
    if(( this->x == v.x ) &&( this->y == v.y ) )
         return false;
    else
         return true;
}

 void rotate(RotMat M)
 {
  float xprime = x * M.cosinus - y * M.sinus;
  float yprime = x * M.sinus + y * M.cosinus; 
        
   x = xprime;
   y = yprime;
 }

 void set(float X , float Y)
 { 
	 x=X;
	 y=Y;
 }

 float value()
 {
  return sqrt(x*x + y*y);
 }

 float square_value()
 {
  return x*x + y*y ;
 }

 void normalize()
 {
	 if( x == 0 && y == 0)
	 return;

	 this->multiply(  1 / this->value() );
 }

 void clear()
 {
	 x=0.0f;
	 y=0.0f;
 }

 void add(Vec2 b)
 {
  x += b.x;
  y += b.y;
 }

 void subtract(Vec2 b)
 {
  x = x - b.x;
  y = y - b.y;
 }

 void multiply(float s)
 {
  x = x * s;
  y = y * s;
 }

};

 // Odcinek
 struct Segment
 {
  Vec2 a;
  Vec2 b;
 };

 // Iloczyn skalarny dwóch wektorów
float Scalar(Vec2 a , Vec2 b);

Vec2 Cross( Vec2 v, float a );

Vec2 Cross( float a, Vec2 v );

float Cross( Vec2 a, Vec2 b );

 // Wektor pomiêdzy dwoma wektorami
Vec2 Vector(Vec2 A , Vec2 B);

  // Odeleg³oœæ dwóch punktów
float distance(Vec2 A,Vec2 B);

 // Kwadrat odleg³oœci dwóch punktów
 float squared_distance(Vec2 A,Vec2 B);

////// funkcja minimum
//float min(float a , float b);
////
//// // funkcja maksimum
//float max(float a , float b);

 // Funkcja zakresu
float clamp(float min , float max , float var);

 //Pitagoras
float Pythagorean( float a , float b );

 Vec2 * intersection( Vec2 A , Vec2 B , Vec2 C , Vec2 D );

 float square(float f);

 float TriangleArea( Vec2 A , Vec2 B , Vec2 C);

