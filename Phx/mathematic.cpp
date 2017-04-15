#include "mathematic.h"
#define NULL 0

float Scalar(Vec2 a , Vec2 b)
 {  
   return a.x * b.x  + a.y * b.y;
 }

Vec2 Cross( Vec2 v, float a )
{
  return Vec2( a * v.y, -a * v.x );
}

Vec2 Cross( float a, Vec2 v )
{
  return Vec2( -a * v.y, a * v.x );
}

float Cross( Vec2 a, Vec2 b )
{
  return a.x * b.y - a.y * b.x;
}

 // Wektor pomiêdzy dwoma wektorami
Vec2 Vector(Vec2 A , Vec2 B)
{
 return Vec2(B.x-A.x,B.y-A.y);
}

  // Odeleg³oœæ dwóch punktów
float distance(Vec2 A,Vec2 B)
 {
   return sqrt( (A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y) );
 }

 // Kwadrat odleg³oœci dwóch punktów
float squared_distance(Vec2 A,Vec2 B)
 {
   return (A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y) ;
 }

// funkcja minimum
float min(float a , float b)
  {
	if( a < b)
		return a;
	else 
		return b;
  }

 // funkcja maksimum
float max(float a , float b)
  {
	if( a < b)
		return b;
	else 
		return a;
  }

 // Funkcja zakresu
float clamp(float min , float max , float var)
 {
   if(var > max)
   return max;

   if(var<min)
   return min;

   return var;
 }

 //Pitagoras
float Pythagorean( float a , float b )
 {
   return sqrt(a*a + b*b);
 }



Vec2 * intersection(Vec2 p1, Vec2 p2, Vec2 p3, Vec2 p4)
{

// Store the values for fast access and easy
// equations-to-code conversion

float x1 = p1.x, x2 = p2.x, x3 = p3.x, x4 = p4.x;
float y1 = p1.y, y2 = p2.y, y3 = p3.y, y4 = p4.y;
 
float d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
// If d is zero, there is no intersection
if (d == 0) return NULL;
 
// Get the x and y
float pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
float x = ( pre * (x3 - x4) - (x1 - x2) * post ) / d;
float y = ( pre * (y3 - y4) - (y1 - y2) * post ) / d;
 
// Check if the x and y coordinates are within both lines
if ( x < min(x1, x2) || x > max(x1, x2) ||
x < min(x3, x4) || x > max(x3, x4) ) return NULL;
if ( y < min(y1, y2) || y > max(y1, y2) ||
y < min(y3, y4) || y > max(y3, y4) ) return NULL;
 
// Return the point of intersection
Vec2* ret = new Vec2();
ret->x = x;
ret->y = y;
return ret;
}

  float square(float f)
  {
   return f*f;
  }

  float TriangleArea( Vec2 A , Vec2 B , Vec2 C)
  {
	  return 0.5f * abs( (B.x - A.x)*(C.y - A.y) - (C.x - A.x)*(B.y - A.y) );
  }

  bool SqrSegmDist(Vec2 SA, Vec2 SB, Vec2 P , float & dist) 
  {

  const float l2 = squared_distance(SA, SB);  // i.e. |w-v|^2 -  avoid a sqrt

  if (l2 == 0.0) return distance(P, SA);   // v == w case

  // Consider the line extending the segment, parameterized as v + t (w - v).
  // We find projection of point p onto the line. 
  // It falls where t = [(p-v) . (w-v)] / |w-v|^2

  const float t = Scalar(P - SA, SB - SA) / l2;

  if (t < 0.0) 
  {
  dist = squared_distance(P, SA);
  return true;       // Beyondthe 'v' end of the segment
  }


  else if (t > 1.0) 
  {
  dist = squared_distance(P, SB);
  return true;       // Beyond the 'w' end of the segment
  }


  const Vec2 projection = SA + (SB - SA) * t;  // Projection falls on the segment

  dist = squared_distance(P, projection);
  return false;

}