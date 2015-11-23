#include "Phx2.h"
#include <GLFW/glfw3.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>


Circle::Circle()
{
 body = NULL;     // Wszystko zeruje
 radius = 0;
 global = false;
 type = circle;
 child = false;
 orient = NULL;
}

Circle::Circle(float r)
{
 body = NULL;   // Przypisuje promieñ
 radius = r;
 global=false;
 type = circle;
 child = false;
 orient = NULL;
}

void Circle::draw()
{
   glPushMatrix();                                         // Odk³adam macierz na stos

   glTranslatef(body->position.x,body->position.y,0);      // Przekszta³cam obecn¹ macierz
   glRotatef(body->angle, 0.0f, 0.0f, 1.0f);

   if(child)
   {
     glTranslatef(orient->pos.x , orient->pos.y , 0);                               // Przekszta³cam podanymi parametrami
     glRotatef(orient->rot , 0.0f, 0.0f, 1.0f);
   }

   double rad , point_x , point_y;                         // zmienne do rysowania ko³a

   glBegin( GL_LINE_LOOP );

	for(int i =0; i < 24; i++)                             // Wyznaczam kolejne 30 punktów po okrêgu
	{
	rad = 2 * M_PI * i / 24;
	point_x = cos(rad) * radius;
	point_y = sin(rad) * radius;

	glVertex2d( point_x ,  point_y);
	}
	glEnd();


	if(body->dynamic == true && child == false )
	{
	glBegin(GL_LINES);

	glVertex2d(0,radius);                                 
	glVertex2d(0,0 );


   glEnd();
	}

   glPopMatrix();                                          // Zdejmuje macierz ze stosu
}

float Circle::area()
{
  return M_PI * radius * radius;            // PIr^2
}

float Circle::inertia()
{
 return (radius*radius*body->mass)/4;      // r^2m/4
}

void Circle::calc_glob()
{
  if(child)
  {
	RotMat m(body->angle );

	Vec2 t = orient->pos;

	t.rotate(m);

    center = body->position + t; 
  }
  else
  {
   center = body->position ;
  }

  global = true;
}



Poly::Poly()
{
 body = NULL;          // Zeruje wszystko
 vertices = NULL;
 GlobNorm = NULL;
 GlobVert = NULL;
 size = 0;
 radius = 0;
 global = false;
 type = polygon ;
 child = false;
 orient = NULL;
}

Poly::Poly(Vec2 * tab , int n)
{
 body = NULL;

 size = n;                      // Pobieram iloœæ wierzcho³ków

 GlobNorm = new Vec2[n];        // Tworzê tablicê na wierzcho³ki i normalne

 GlobVert = new Vec2[n];

 vertices = new Vec2[n];

  for(int i=0;i<n;i++)
 {
  vertices[i] = tab[i];         // Kopiujê wierzcho³ki
 }


 Vec2 center(0,0);                        // Zmienna na œrodek ciê¿koœci wielok¹tu
 Vec2 * centers = new Vec2[n-2];          // Tablica œrodków poszczególnych trójk¹tów (triangulizacja wielok¹tu )

 float area = 0.0f;                       // Pole figury
 float * areas = new float[n-2];          // Pola trójk¹tó ( triangulizacja )

 // Triangulizacja 
 for (int i=2; i < size ; i++)
 {
		centers[i - 2] =  Vec2(  (vertices[0].x+vertices[i-1].x+vertices[i].x)/3 , (vertices[0].y+vertices[i-1].y+vertices[i].y)/3  );  // Œriednia kolejnych wierzcho³ków

		areas[i-2] = TriangleArea(vertices[0] , vertices[i] , vertices[i-1] );    // Pole trójk¹ta

		area += areas[i-2] ;
 }

 for (int i=0; i < n-2 ; i++)
 center = center + centers[i]*areas[i];   // Sumujemy œrodki trójk¹tów pomno¿one przez ich pole

 center = center / area;  // Dzielimy przez ³¹czne pole

 delete[] areas;
 delete[] centers;

 // Przesuwamy k¹zdy wierzcho³ek tak ¿eby œrodek ciê¿koœci by³ punktem (0,0)
 for (int i=0; i<size; i++)
 vertices[i] = vertices[i] - center;

 // Szukam promienia okrêgu opisuj¹cego
 radius = 0;

 float r;

 for(int i=0;i<size;i++)                                                  // Szukam minimalnego promienia okrêgu okalaj¹cego wielok¹t
 {
  r = vertices[i].x * vertices[i].x + vertices[i].y * vertices[i].y;      // Liczê kwadraty odleg³oœci wierzcho³ków od œrodka

  if( r > radius)                                                       // Porównuje je
  radius = r;                                                           // Przepisuje wiêksz¹ wartoœæ
 }

 radius = sqrt(radius);                                               // Na sam koniec pierwiastkuje minimaln¹ wartoœæ

 global = false;

 type = polygon;
 child = false;
 orient = NULL;
}

Poly::~Poly()
{
	delete[] GlobNorm ;          // Usuwam dynamicznie alokowan¹ pamiêæ

    delete[] GlobVert ;

    delete[] vertices ;
}

void Poly::draw()
{
   glPushMatrix();

   glTranslatef(body->position.x,body->position.y,0);
   glRotatef(body->angle, 0.0f, 0.0f, 1.0f);

   if(child)
   {
   glTranslatef(orient->pos.x , orient->pos.y,0);                             
   glRotatef(orient->rot, 0.0f, 0.0f, 1.0f);
   }

   glBegin( GL_LINE_LOOP );

	for(int i=0; i < size ; i++)
	{
	 glVertex2f(vertices[i].x  , vertices[i].y );
	}

	if(body->dynamic == true && child == false )       // Jeœli cia³o jest dynamiczne to rysuje orientacje
	{

	glVertex2f(vertices[0].x  , vertices[0].y );

	glVertex2f(0,0);

	}

   glEnd();

   glPopMatrix();
}

float Poly::area()
{
  float s = 0;           // Zmienna na pole

  for(int i=0;i<size-2;i++)      // Liczy pole sumuj¹c pola trójk¹tów utworzonych za pomoc¹ kolejnych przek¹tnych i boków
  {
	s += TriangleArea( vertices[0] , vertices[i+1] , vertices[i+2] );
  }

  return s; 
}

float Poly::inertia()
{
  float I = 0.0f;       // Tego zupe³nie nie rozumiem , zróbi³em tylko optymalizacje

  float a = 0.0f;

  float ai;

  for (int i=0; i<size; i++)
  {
	ai = abs(vertices[(i+1)%size].x*vertices[i].y - vertices[i].x*vertices[(i+1)%size].y);

	I += ai * (Scalar(vertices[(i+1)%size], vertices[(i+1)%size]) + Scalar(vertices[(i+1)%size], vertices[i]) + Scalar(vertices[i], vertices[i]));

	a += ai;
  }

  I *= body->mass/6.0f/a;

  return I;
}

void Poly::calc_glob()  // Liczy dane globalne
{
 // Kopiuje wierzcho³ki
 for(int i=0;i<size;i++)
 GlobVert[i] = vertices[i];  

 // Licze globalne pozycje

 if(child)
 {
 RotMat mr(orient->rot);

	  for(int i=0;i<size;i++)            // Obracam i przesuwam ka¿dy wierzcho³ek
	 {
	   GlobVert[i].rotate(mr);
	   GlobVert[i].add(orient->pos);
	 }
 }

 RotMat m(body->angle);             // Tworzê macierz rotacji

 for(int i=0;i<size;i++)            // Obracam i przesuwam ka¿dy wierzcho³ek
 {
   GlobVert[i].rotate(m);
   GlobVert[i] = GlobVert[i] + body->position;
 }

 // Licze globalne normalne

 for(int i=0;i<size;i++)   
 {
  GlobNorm[i] = Vec2( GlobVert[(i+1)%size].y - GlobVert[i].y ,
	                  GlobVert[i].x - GlobVert[(i+1)%size].x );

  GlobNorm[i].normalize();   
 }

 // Licze pozycje

 if(child)
  {
	RotMat m(body->angle );

	Vec2 t = orient->pos;

	t.rotate(m);

    center = body->position + t; 
  }
  else
  {
   center = body->position ;
  }

  global = true;
}

bool IsPointOnSegment(Vec2 PointA, Vec2 PointB, Vec2 point) 
{
   const float EPSILON = 0.001f;

   if( point.x < std::min( PointA.x , PointB.x) )
   return false;

   if( point.x > std::max( PointA.x , PointB.x) )
   return false;

   if( point.y < std::min( PointA.y , PointB.y) )
   return false;

   if( point.y > std::max( PointA.y , PointB.y) )
   return false;

   if( ( PointB.x - PointA.x  == 0 ) && ( PointB.x == point.x ) )
   return true;

   if( ( PointB.y - PointA.y  == 0 ) && ( PointB.y == point.y ) )
   return true;

   float a = (PointB.y - PointA.y) / (PointB.x - PointA.x);
   float b = PointA.y - a * PointA.x;
   if ( fabs(point.y - (a*point.x+b)) < EPSILON)
   {
       return true;
   }

   return false;
}

bool Poly::IsPointInside(Vec2 p)
{
if(!global)  // Sprawdzam aktualnoœæ danych
calc_glob();

  /* Kod z internetu , zoptymalizowany super-mega sztuczkami c++ 
   Ogólna idea to sprawdziæ czy dla ka¿dego boku wielok¹ta , podany punkt znajduje siê potej samej stronie co reszta wierzcho³ków 
   Jeœli dla któregoœ nie to punkt nie nale¿y do wielok¹tu */

 for(int i=0 ; i<size;i++)
 if( IsPointOnSegment(GlobVert[i],GlobVert[(i+1)%size] , p) )
 return true;


  int   i, j=size-1 ;
  bool  oddNodes = false;

  for (i=0; i<size; i++)
  {
    if (  (GlobVert[i].y <= p.y && GlobVert[j].y >= p.y ||   GlobVert[j].y <= p.y && GlobVert[i].y >= p.y )   &&  (GlobVert[i].x <= p.x || GlobVert[j].x <= p.x)   )
	{
      oddNodes^=(GlobVert[i].x + (p.y-GlobVert[i].y)/(GlobVert[j].y-GlobVert[i].y)*(GlobVert[j].x-GlobVert[i].x)<p.x); 
	}

    j=i; 
  }

  return oddNodes; 
}





Complex::Complex()
{
 childs = NULL;
 size = 0;
 type = complex;
 child = false;
 orient = NULL;
}

Complex::Complex(Child * ch , int n)
{
 childs = new Child[n];

 size = n;

 float r = 0;            // Szukam maksymalnego promienia opisuj¹cego

 for(int i=0;i<n;i++)
 {
  childs[i] = ch[i];
  childs[i].sh->child = true;
  childs[i].sh->orient = &childs[i].or;

  if( r < childs[i].sh->radius + childs[i].or.pos.value() );
  r = childs[i].sh->radius + childs[i].or.pos.value() ;
 }

 radius = r;

 type = complex;
 child = false;
 orient = NULL;
}

Complex::~Complex()
{
 for(int i=0;i<size;i++)
 {
  delete childs[i].sh;
 }

 delete[] childs;
}

void Complex::draw()
{
 for(int i=0;i<size;i++)
 {
   childs[i].sh->draw();
 }
}

float Complex::area()
{
 float a = 0.0f;

 for(int i=0;i<size;i++)
 {
  a += childs[i].sh->area();
 }

	return a;
}

float Complex::inertia()
{
 //calculate new moment of inertia
	float I = 0.0f;
	for (int i=0; i<size; i++)
	{
		I += childs[i].sh->inertia()  + (childs[i].sh->area() * body->material->density   ) * Scalar(childs[i].or.pos , childs[i].or.pos);
	}

	return I;
}

void Complex::calc_glob()  // Liczy dane globalne
{
 for(int i=0;i<size;i++)
 {
  childs[i].sh->calc_glob();
 }
 global = true;
}



Body::Body()
{
 dynamic = false;

 angle = 0;
 omega = 0;

 mass = 0;
 inv_mass = 0;

 inertia = 0;
 inv_inertia = 0;

 torque = 0;

 shape = NULL;

 material = NULL;
}

Body::Body( bool Dynamic , float X , float Y ,float Angle , Shape * sh ,  Material * m)
{
 sh->body = this;

 if(sh->type == complex)
 {
   Complex * com = static_cast<Complex*>(sh);

   for(int i=0;i< com->size ;i++)
   {
	com->childs[i].sh->body = this;
   }
 }

 dynamic = Dynamic;

  material =  m;

 position.x = X;
 position.y = Y;

 angle = Angle;

 if(dynamic)
 {

 mass = sh->area() * m->density;
 inv_mass = 1 / mass;

 inertia = sh->inertia();
 inv_inertia = 1 / inertia;
 }
 else
 {
 mass = 0;
 inv_mass = 0;
 inertia=0;
 inv_inertia=0;
 velocity = Vec2(0,0);
 }

 omega = 0;
 torque = 0;

 epsilon = 0.0f;

 shape    = sh;
}

Body::~Body()
{
 delete shape;
}

void Body::ApplyImpulse(Vec2 impulse , Vec2 contact)
{
 if( dynamic == false )
 return;


 Imp.add(impulse);
 epsilon +=  Cross(contact , impulse );
}

void Body::ApplyForce(Vec2 f )
{
  force.add(f);
}

void Body::CalcForces(float delta)
{
 if( dynamic == false )
 return;

 velocity = velocity + (force * inv_mass ) * (delta/2) ;
 omega = omega + ( torque * inv_inertia ) * (delta/2) ;
}

void Body::CalcPosition(float delta , bool damping)
{
  position.add(velocity * delta);

  angle +=  ( (omega * 360)/ (2*M_PI) ) * delta;

  if(damping)
  {
  omega = omega * 0.995f;
  velocity.multiply(0.998f);
  }

  if( velocity.x == 0 && velocity.y == 0 && omega == 0 )
  shape->global = true;
  else
  shape->global = false;
}

void Body::CalcSpeed()
{
  if( dynamic == false )
  return;

  velocity.add( Imp * inv_mass );
  omega += epsilon * inv_inertia ;

  Imp.clear();
  epsilon = 0.0f;
}

void Body::translate(Vec2 v)
{
  position.add(v);
  shape->global = false;
}



Joint::Joint()
{
 A = NULL;
 B = NULL;

 pA.clear();
 pB.clear();

 distance = 0;
}

Joint::Joint(Body * a , Body * b, float d)
{
 A = a;
 B = b;

 pA.clear();
 pB.clear();

 distance = d;
}

Joint::Joint(Body * a , Vec2 PA , Body * b , Vec2 PB , float d)
{
 A = a;
 B = b;

 pA = PA;
 pB = PB;

 distance = d;
}

void Joint::update(float delta)
{

	Vec2 pa = pA;
	Vec2 pb = pB;

	RotMat mA(A->angle);
	RotMat mB(B->angle);

	pa.rotate(mA);
	pb.rotate(mB);

	pa.add(A->position);
	pb.add(B->position);


	Vec2 ra = pa - A->position;
	Vec2 rpa = Vec2(-ra.y, ra.x);

	Vec2 rb = pb - B->position;
	Vec2 rpb(-rb.y, rb.x);


	float d = distance - (pa-pb).value();

	Vec2 n = pa-pb;

	if (n.value() == 0.0f)
		return;

	n.normalize();

	//position correction
	float s = d / (A->inv_mass + B->inv_mass );

	if(A->dynamic == true )
	A->translate(n * s * A->inv_mass );
	

	if(B->dynamic == true )
	B->translate( n * s * B->inv_mass * -1);

	//relative velocities
	Vec2 v1 = A->velocity + rpa*A->omega;
	Vec2 v2 = B->velocity + rpb*B->omega;

	Vec2 v = v1-v2;

	//calculate impulse
	float j = -Scalar(v, n) / (A->inv_mass  + B->inv_mass  + Scalar(rpa, n)*Scalar(rpa, n)*A->inv_inertia  + Scalar(rpb, n)*Scalar(rpb, n)*B->inv_inertia );

	

	//apply impulse
	if(A->dynamic == true )
	{
	A->velocity.add( n * A->inv_mass * j );
	A->omega += Scalar(rpa, n*j) * A->inv_inertia ;
	}

	if(B->dynamic == true )
	{
	B->velocity.add (n * B->inv_mass * j * -1 );
	B->omega -= Scalar(rpb, n*j) * B->inv_inertia ;
	}
	/*
  Vec2 AB = Vector(A->position , B->position );             // Wektor AB

  float translation = AB.value() - distance;                // wymagana translacja cia³

  AB.normalize();                                           // normalizuje wektor

  Vec2 RelVel = B->velocity - A->velocity  ;                // Wypadkowa prêdkoœæ cia³

  float vel = Scalar(RelVel , AB) + translation;
        
 vel = vel / ( A->inv_mass + B->inv_mass );

 AB = AB * vel;
 
 A->ApplyImpulse(AB);
 B->ApplyImpulse(AB * -1);
 */
}

void Joint::draw()
{
	glColor3ub(0,200,0);

	glBegin( GL_LINES );

	glVertex2f(A->position.x  + pA.x   , A->position.y + pA.y);
	glVertex2f(B->position.x  + pB.x   , B->position.y + pB.y);


	glEnd();

}


void Collision::Initialize(float delta,Vec2 Gravity)
{
  // Calculate average restitution
  e = (A->material->restitution + B->material->restitution )/2;

  // Calculate static and dynamic friction
  sf = sqrt( A->material->static_friction * B->material->static_friction );
  df = sqrt( A->material->dynamic_friction * B->material->dynamic_friction );

  for(int i = 0; i < contacts_num ; ++i)
  {
    // Calculate radii from COM to contact
    Vec2 ra = contacts[i] - A->position;
    Vec2 rb = contacts[i] - B->position;

    Vec2 rv = B->velocity + Cross( B->omega, rb ) -
              A->velocity - Cross( A->omega, ra );


    if(rv.square_value() < (delta * Gravity.square_value() ) )
     e = 0.0f;
  }
}


bool FastCheck(Body * A , Body * B)
{
  if( Vector( A->position  , B->position ).square_value() > (A->shape->radius + B->shape->radius) * (A->shape->radius + B->shape->radius) )     // Jeœli kwadrat d³ugoœci wektora jest wiêkszy ni¿ kwadrat sumy promieni 
  return false; 
 
  return true;
}


Phx::Phx()
{
	Gravity = Vec2(0,-300);  // Ustawiamy wektor grawitacji
	gravitation = true;   // Grawitacje
	friction = true;      // Tarcie
	damping = true;

	percent = 0.5f;        // Wspó³czynniki korekcji pozycji  
	treshhold = 0.05f;

	


	DebugDraw = false;
}

Phx::~Phx()
{
 for(int i=0;i<BodyList.size();i++)          // Usuwamy wszystkie obiekty
 delete BodyList[i];

 for(int i=0;i<Collisions.size();i++)        // Usuwamy wszystkie pozosta³e kolizje
 delete Collisions[i];

 for(int i=0;i<Joints.size();i++)            // Usuwamy wszystkie pozosta³e po³¹czenia
 delete Joints[i];
}

void Phx::DeleteBodies()
{
  for(int i=0;i<BodyList.size();i++)         // Usuwamy wszystkie obiekty
 delete BodyList[i];

 for(int i=0;i<Collisions.size();i++)        // Usuwamy wszystkie pozosta³e kolizje
 delete Collisions[i];

 for(int i=0;i<Joints.size();i++)            // Usuwamy wszystkie pozosta³e po³¹czenia
 delete Joints[i];

 BodyList.clear();
 DynamicList.clear();
 StaticList.clear();
 Collisions.clear();
 Joints.clear();
}

void Phx::Step(float delta)
{
	if(gravitation)           // Dodajemy grawitacje
	ApplyGravitation();     

	for(int i=0;i<DynamicList.size();i++)  // Liczymy si³y
	DynamicList[i]->CalcForces(delta);

	SearchForCollisions();     // Szukamy kolizji

	for(int i=0;i<Collisions.size();i++)      // Liczymy wstêpne dane
    Collisions[i]->Initialize(delta,Gravity);

	for(int i=0;i<10;i++)
	{
	 for(int i=0;i<Collisions.size();i++)
     Resolve(Collisions[i],delta);                             // Rozwi¹zujemy wszystkie wykryte kolizje

	ResolveJoints(delta);      // Rozwi¹zujemy po³¹czenia

	}

	PositionalCorrection();    // Korygujemy pozycje

	MoveBodies(delta);         // Poruszamy obiekty

	for(int i=0;i<DynamicList.size();i++)
	DynamicList[i]->CalcForces(delta);

	for(int i=0;i<DynamicList.size();i++)
	{
	DynamicList[i]->force.clear();
	DynamicList[i]->torque = 0;
	}
}

void Phx::ApplyGravitation()
{
  for(int i=0;i<DynamicList .size();i++)                                  // Iterujemy po aktywnych cia³ach
  {
    DynamicList[i]->ApplyForce(Gravity * DynamicList[i]->mass);           // F = mg
  }
}

void Phx::SearchForCollisions()
{
  for(int i=0;i<Collisions.size();i++)        // Usuwamy wszystkie pozosta³e kolizje
  delete Collisions[i];

  Collisions.clear();

  if(BodyList.size() == 0)
  return;

  for(int a=0 ; a < BodyList.size()-1 ; a++)             // Dla ka¿dej pary cia³ A i B 
  {
	for(int b=a+1 ; b<BodyList.size() ; b++)
	{
	  if(BodyList[a]->dynamic == false && BodyList[b]->dynamic == false) 
	  continue;

	  if(FastCheck(BodyList[a] , BodyList[b]) )
	  Check( BodyList[a]->shape , BodyList[b]->shape );          
	 
	}
  }
}

void Phx::Check( Shape * A , Shape * B )
{
	Collision * c ;   // Tworzymy strukturê danych o kolizji

	if( A->type == circle && B->type == circle )   // Sprawdzamy typy obiektów
	{
		Circle * circleA = static_cast<Circle*>(A);
        Circle * circleB = static_cast<Circle*>(B);

		if( A->global == false )
		circleA->calc_glob();

		if( B->global == false )
		circleB->calc_glob();

		c = CircleVsCircle(circleA , circleB);

		if( c != NULL )
		Collisions.push_back(c);         // Jeœli tak to zapisujemy t¹ kolizjê

		return;                          // Koñczymy funkcjê
	}

	if( A->type == polygon && B->type == polygon )
	{
	   Poly * polygonA = static_cast<Poly*>(A);        // Rzutuje wskaŸniki
       Poly * polygonB = static_cast<Poly*>(B);

		if( A->global == false )
		polygonA->calc_glob();

		if( B->global == false )
		polygonB->calc_glob();

		c = PolygonVsPolygon(polygonA,polygonB);

		if( c != NULL )
		Collisions.push_back(c);        

		return;           
	}

	if( A->type == polygon && B->type == circle )
	{
		Poly * polygonA = static_cast<Poly*>(A);            // Rzutuje wskaŸniki
	    Circle  * circleB = static_cast<Circle*>(B);

		if( A->global == false )
		polygonA->calc_glob();

		if( B->global == false )
		circleB->calc_glob();

		c = PolygonVsCircle(polygonA,circleB);

		if( c != NULL )
		Collisions.push_back(c);        

		return;   
	}

	if( A->type == circle && B->type == polygon )
	{
		Poly * polygonB = static_cast<Poly*>(B);            // Rzutuje wskaŸniki
	    Circle  * circleA = static_cast<Circle*>(A);

		if( A->global == false )
		circleA->calc_glob();

		if( B->global == false )
		polygonB->calc_glob();

		c = PolygonVsCircle(polygonB,circleA);

		if( c != NULL )
		Collisions.push_back(c);        

		return; 
	}

	if( A->type == complex )
	{
	   Complex * com = static_cast<Complex*>(A);

	   if(com-> global == false )
	   com->calc_glob();

	   for(int i=0;i<com->size;i++)
	   {
	   Check( com->childs[i].sh , B );
	   }
	}

	if( B->type == complex )
	{
	   Complex * com = static_cast<Complex*>(B);

	   if(com-> global == false )
	   com->calc_glob();

	   for(int i=0;i<com->size;i++)
	   Check( com->childs[i].sh , A );
	}

}

void Phx::Resolve(Collision * col,float delta)
{
   Body * A = col->A ;                                // Pobieramy dane z struktury kolizji
   Body * B = col->B ;                                // WskaŸniki obiektów
   Vec2 normal = col->normal;                         // Normalna

   Vec2 rv ;

 


   for(int i=0;i<col->contacts_num ;i++)
   {

   // Calculate radii from COM to contact
   Vec2 ra = col->contacts[i] - A->position;
   Vec2 rb = col->contacts[i] - B->position;

	   rv = B->velocity + Cross( B->omega , rb ) -
              A->velocity - Cross( A->omega, ra );                     // Wypadkowa prêdkoœæ 


   if(rv.square_value() < (delta * Gravity.square_value() ) )
   col->e = 0.0f;
   else
   col->e = (A->material->restitution + B->material->restitution )/2;

   float velAlongNormal = Scalar(rv,normal);          // Skalar wypadkowej prêdkoœci i normalnej
 
   if(velAlongNormal > 0)                             // Jeœli prêdkoœci d¹¿¹ do roz³¹czenia siê cia³ 
   continue;                                            // Koñczymy funkcjê

   float raCrossN =   Cross( ra, normal );
   float rbCrossN =   Cross( rb, normal );

   float invMassSum = A->inv_mass + B->inv_mass + square( raCrossN ) * A->inv_inertia  +  square( rbCrossN ) * B->inv_inertia ;

    float j = -(1.0f + col->e) * velAlongNormal;
    j /= invMassSum;
    j /= col->contacts_num;
 
	Vec2 impulse = normal * j;
    A->ApplyImpulse( impulse * -1, ra );
    B->ApplyImpulse(  impulse, rb );

	
  
	if(!friction)
	continue;

	

    rv = B->velocity + Cross( B->omega , rb ) -
              A->velocity - Cross( A->omega, ra );

    Vec2 t = rv - (normal * Scalar( rv, normal ));
    t.normalize( );

    // j tangent magnitude
    float jt = -Scalar( rv, t );
    jt /= invMassSum;
    jt /= col->contacts_num;




    // Don't apply tiny friction impulses
	if( abs(jt) < 0.1 )
    continue;

    // Coulumb's law
    Vec2 tangentImpulse;
    if(std::abs( jt ) < j * col->sf)
      tangentImpulse = t * jt;
    else
      tangentImpulse = t * -j * col->df;

    // Apply friction impulse
    A->ApplyImpulse( tangentImpulse * -1, ra );
    B->ApplyImpulse(  tangentImpulse, rb );

   }

   A->CalcSpeed();
   B->CalcSpeed();


}

void Phx::PositionalCorrection()
{
  for(int i=0;i<Collisions.size();i++)
  Correct(Collisions[i]);                // Dokonujemy korekty po³o¿enia

}

void Phx::Correct(Collision * c)
{
   if(c->penetration < treshhold)      // Jeœli penetracja jest mniejsza ni¿ próg
   return;                             // Koñczymy funkcjê

   //   Korekcja   =  normalna  * procent * penetracja     / ( suma odwrotnoœci mas)
   Vec2 correction =  c->normal * percent * c->penetration / (c->A->inv_mass + c->B->inv_mass);


   c->B->translate( correction * c->B->inv_mass);    // Mno¿ymy przez odwrócon¹ masê cia³a  ,  l¿ejsze obiekty bêd¹ przesuniête bardziej ni¿ ciê¿kie 
                                                 
   correction = correction * -1 ;                // Odwracamy wektor

   c->A->translate( correction * c->A->inv_mass );   // Przesuwamy drugie cia³o 

   c->A->shape->global=false;
   c->B->shape->global=false;
}

void Phx::MoveBodies(float delta)
{
  for(int i=0;i<DynamicList .size();i++)    // Iterujemy po dynamicznych cia³ach
  {
    DynamicList[i]->CalcPosition(delta,damping );            // Poruszamy wszystkie dynamiczne cia³a
  }
}

void Phx::ResolveJoints(float delta)
{
 for(int i=0;i<Joints.size();i++)
 {
  Joints[i]->update(delta);
 }
}

void Phx::AddBody( Body * b )
{
  BodyList.push_back(b);

  if(b->dynamic)
  DynamicList.push_back(b);
  else
  StaticList.push_back(b);
}

void Phx::DrawAll()
{
	glColor3ub(255,0,255);
	for(int i=0;i<StaticList.size();i++)
	StaticList[i]->shape->draw();

	glColor3ub(0,255,255);
	for(int i=0;i<DynamicList.size();i++)
	DynamicList[i]->shape->draw();

	glColor3ub(255,255,0);
	for(int i=0;i<Joints.size();i++)
	Joints[i]->draw();

	glColor3ub(255,0,0);
	glPointSize(4);

	if(!DebugDraw)
	return;

	glBegin( GL_POINTS );

	for(int i=0;i<Collisions.size();i++)
	for(int z=0;z<Collisions[i]->contacts_num;z++)
	glVertex2f( Collisions[i]->contacts[z].x , Collisions[i]->contacts[z].y )  ;

	glEnd();
}






Collision * CircleVsCircle(Circle * A , Circle * B)
{
  Vec2 normal = Vector( A->center  , B->center );          // Wektor od A do B

  if(normal.square_value() > square(A->radius + B->radius ) )
  return NULL;

  Collision * c = new Collision ;

  c->A = A->body;
  c->B = B->body;


  float d = normal.value();            // Obliczamy odleg³oœæ kó³ ( g³upi pierwiastek :( )
 
  if(d != 0)                           // Jeœli odleg³oœæ nie wynosi zero
  {
    c->penetration = A->radius +  B->radius - d;  // Obliczamy penetracje i j¹ zapisujemy do struktury kolizji
 
	normal.normalize();                // Normalizujemy normaln¹

    c->normal = normal;                // zapisujemy do struktury

	c->contacts_num = 1;

	c->contacts[0] = A->center + normal * A->radius ;

    return c;                       // Zwracamy prawdê
  }
  else                                 // Ko³a na tej samej pozycji
  {
    // Jesteœmy w dupie , wybieramy randomowe wartoœci

    if(A->radius > B->radius )    // Chyba lepiej podaæ wiêksz¹ penetracje
    c->penetration = A->radius;
	else
	c->penetration = B->radius;

    c->normal = Vec2( 1, 0 );       //  normalna w prawo Korwin style

	c->contacts_num = 1;

	c->contacts[0] = A->center ;
    
	return c;
  }
}

Collision * PolygonVsPolygon(Poly * A , Poly * B)
{

    int edgesA = A->size;                                    // Pobieram iloœæ wierzcho³ków
    int edgesB = B->size;

    float minPenetration = INT_MAX;                                 // Zmienna na minimaln¹ penetracje
	float Penetration;                                              // Zmienna na aktualn¹ penetracje

    Vec2 normal ;                                                   // Poszukiwana normalna

	Vec2 axis;                                                      // Potencjalna oœ separacji

	float minA = 0; float maxA = 0;                                 // Zmienne na rzuty obu wielok¹tów
	float maxB = 0; float minB = 0; 


    for(int i = 0; i < edgesA + edgesB; i++)                       // Iteruje po wszystkich krawêdziach
	{
        if (i < edgesA)                                           
		{
            axis = A->GlobNorm[i];                            // Pobieramy normaln¹
        } 
		else 
		{
            axis = B->GlobNorm[i - edgesA];
        }

        ProjectPolygon(axis, A, minA, maxA);                // Rzutujemy oba wielok¹ty na oœ
        ProjectPolygon(axis, B, minB, maxB);

		Penetration = ProjectionDistance(minA, maxA, minB, maxB);  // Liczy odleg³oœæ projekcji

        if ( Penetration > 0)                                      // Jeœli odleg³oœæ wiêksza od zera nie ma kolizji
        return NULL;                                               // Wystarczy jedna oœ separacji

        Penetration = -Penetration;                            // Zmieniam znak penetracji

		// Teraz sprawdzam czy jest to najmniejsza mo¿liwa penetracja

        if (Penetration < minPenetration)
		{
            minPenetration = Penetration;
            normal = axis;

            if ( Scalar(Vector( B->center , A->center ),normal)  > 0)  // Sprawdzam czy normalna ma dobry zwrot
            normal = normal * -1;
        }
    }



    // Skoro znalaz³em ju¿ wszystkie dane uzpe³niam dane o kolizji


	Collision * c = new Collision;

	c->contacts_num = 0;


	// Szukam kolizji wierzcho³ków


	//Kolizje wierzcho³ek B wielok¹t A
	for(int b=0; b < B->size;b++)
	{ 
		  if( A->IsPointInside(B->GlobVert[b] ) )
		  {
			   if( c->contacts_num != 2 )
			   {
				   c->contacts_num++;
				   c->contacts[c->contacts_num-1] = B->GlobVert[b];
			   }
			   else
			   break;
		 }
	}

	//Kolizje wierzcho³ek A wielok¹t B
	if( c->contacts_num != 2 )
	for(int a=0; a < A->size;a++)
	{ 
	  if( B->IsPointInside(A->GlobVert[a] ) )
	  {
	        if( c->contacts_num != 2 )
			{
				c->contacts_num++;
				c->contacts[c->contacts_num-1] = A->GlobVert[a];
			}
			else
			break;
	 }
	}


	c->A = A->body;
	c->B = B->body;

	c->normal = normal;
	c->penetration = minPenetration;
  

    return c;
}

Collision * PolygonVsCircle(Poly * A , Circle * B )
{
    int edges = A->size;                                // Pobieramy iloœæ wierzcho³ków

    float minPenetration = INT_MAX;                                 // Zmienna na minimaln¹ penetracje
	float Penetration;                                              // Zmienna na aktualn¹ penetracje

    Vec2 normal ;                                                   // Poszukiwana normalna
	Vec2 axis;                                                      // Potencjalna oœ separacji

	float minA = 0; float maxA = 0;                                 // Zmienna na rzuty obu wielok¹tów
	float maxB = 0; float minB = 0; 


	// Poszukujemy osi sepracji wzglêdem wszystkich boków

   for(int i=0 ; i<edges ; i++)
  {
        ProjectPolygon(A->GlobNorm[i] , A , minA, maxA);                // Rzutuje figury
        ProjectCircle (A->GlobNorm[i] , B , minB, maxB);

		Penetration = ProjectionDistance(minA, maxA, minB, maxB);  // Pobieram odleg³oœæ projekcji

	    
        if (Penetration > 0)                                       // Jeœli odleg³oœæ jest wiêksza od zera cia³a nie koliduj¹ 
        return NULL;

        Penetration = -Penetration;                                // Zmieniam znak

        if (Penetration < minPenetration)                          // Sprawdzam czy jest to minimalna penetracja
		{
            minPenetration = Penetration;                          // Zapamiêtuje dane
            normal = A->GlobNorm[i];

            if ( Scalar( Vector(B->center , A->center ) , normal )  > 0)                            // Sprawdzam zwrot normalnej
             normal = normal * -1;
	    }
  }


   // Szukamy najbli¿szego wierzcho³ka

   int ClosestVert;
   float MinDist = FLT_MAX; 

    for(int i=0; i <A->size ; i++)
  {
	if( (B->center - A->GlobVert[i]).square_value() < MinDist )
	{
	 ClosestVert = i;
	 MinDist = (B->center - A->GlobVert[i]).square_value();
	}
  }


  // Sprawdzamy czy przez ten wierzcho³ek przechodzi oœ separacji

	axis = B->center - A->GlobVert[ClosestVert];

	axis.normalize();

	ProjectPolygon(axis, A, minA, maxA);              // Rzutuje figury
    ProjectCircle(axis,  B, minB, maxB);

    Penetration = ProjectionDistance(minA, maxA, minB, maxB);  // Pobieram odleg³oœæ projekcji

	    
    if (Penetration > 0)               // Jeœli odleg³oœæ jest wiêksza od zera cia³a nie koliduj¹ 
    return NULL;

    Penetration = -Penetration;  // Zmieniam znak

    if (Penetration < minPenetration)
    {
        minPenetration = Penetration;
        normal = axis;

        if ( Scalar( Vector(B->center , A->center) , normal )  > 0)                            // Sprawdzam zwrot normalnej
        normal = normal * -1;
	}

	

	// Uzupe³niamy dane o kolizji


	Collision * c = new Collision;

	c->A = A->body ;
	c->B = B->body;

	c->normal = normal;
	c->penetration = minPenetration;

	c->contacts[0] = B->center - normal * B->radius ;

	c->contacts_num = 1;
    
    return c;
}





void ProjectPolygon(Vec2 axis, Poly * polygon,  float & min,  float & max)         // Rzutowanie wielok¹ta na oœ
{
    float dotProduct = Scalar(axis,polygon->GlobVert[0]); 

    min = dotProduct;
    max = dotProduct;

    for (int i = 1; i < polygon->size; i++)
	{
        dotProduct = Scalar(axis,polygon->GlobVert[i]);

        if (dotProduct < min) 
		{
           min = dotProduct;
		   continue;
        } 

        if (dotProduct> max) 
		{
           max = dotProduct;
        }
    }
}

void ProjectCircle(Vec2 axis, Circle * circle,  float & min,  float & max)         // Rzutowanie okregu na oœ
{
    float dotProduct = Scalar(axis, circle->center ); 

    min = dotProduct - circle->radius ;
    max = dotProduct + circle->radius;
}

float ProjectionDistance(float minA, float maxA, float minB, float maxB)            // Szukanie odleg³oœci projekcji na oœ
{
    if (minA < minB) 
	{
        return minB - maxA;
    } 
	else 
	{
        return minA - maxB;
    }
}