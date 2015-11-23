#pragma once

#include <vector>
#include <iostream>
#include "Mathematic.h"

using std::vector;



// Materia�
struct Material
{
  float density;                // G�sto��
  float restitution;            // Wsp�czynnik odbicia
  float static_friction;        // Tarcie Statyczne
  float dynamic_friction;       // Tarcie dynamiczne
};

// Rodzaje kszta�t�w
enum ShapeType
{
  circle,               // Ko�o
  polygon,              // Wielok�t ( tylko wypuk�y)
  complex               // Z�o�one cia�o
};

// Prototypy
class Body;
struct Orient;

// Klasa bazowa kszta�tu
class Shape
 {
   public:

   Body * body;                 // Wska�nik na cia�o posiadaj�ce ten kszta�t

   bool global;                 // Flaga aktualno�ci danych globalnych

   float radius;                // Promie� okr�gu opisuj�cego

   ShapeType type;              // Rodzaj kszta�tu

   bool child;                  // Flaga czy jest dzieckiem innego kszta�tu

   Orient * orient;             // Wska�nik na orientacje wynikaj�c� z bycia dzieckiem 

   Vec2 center;

   virtual ~Shape(){};
   virtual void  draw() = 0;    // Interfejs
   virtual float area() = 0;
   virtual float inertia() = 0;
   virtual void calc_glob()= 0;
 };



// Ko�o
class Circle : public Shape
{
  public :

	Circle();
	Circle(float r);
	~Circle(){};

	void  draw();
	float area();
	float inertia();
	void  calc_glob();
};

// Wielok�t ( tylko wypuk�y )
class Poly : public Shape
{
  public:
  Vec2 * vertices;     // tablica lokalnych wierzcho�k�w

  Vec2 * GlobVert;     // tablica globalnych wierzcho�k�w
  Vec2 * GlobNorm;     // tablica globalnych normalnych

  int size;            // ilo�� wierzcho�k�w

  Poly();
  Poly(Vec2 * tab , int n);
  ~Poly();

  void  draw();
  float area();
  float inertia();
  void  calc_glob();  

  bool  IsPointInside(Vec2 p );
};

struct Orient
{
 Vec2 pos;
 float rot;
};

struct Child
{
 Shape * sh;

 Orient or;
};

class Complex : public Shape
{
public:
 Child * childs;
// Child * GlobChilds;

 int size;

  Complex();
  Complex(Child * ch , int n);
  ~Complex();

  void  draw();
  float area();
  float inertia();
  void  calc_glob();  

  void draw(Vec2 t , float r){};
  void calc_glob(Vec2 t , float r){};
};



// Cia�o fizyczne
class Body
{
  public:

  bool dynamic;         // dynamika cia�a

  Vec2 position;        // Pozycja w przestrzeni
  Vec2 velocity;        // Pr�dko��

  float angle;          // K�t obrotu
  float omega;          // Pr�dko�� k�towa

  float mass;           // Masa cia�a
  float inv_mass;       // Odwr�cona masa

  float inertia;        // Bezw�adno��
  float inv_inertia;    // Odwr�cona bezw�adno��

  Vec2 force;           // Si�a
  float torque;         // Moment si�y

  Shape * shape;        // Kszta�t

  Vec2 Imp;
  float epsilon;

  Material * material;  // Materia�

  public:

   Body();
   Body( bool Dynamic , float X , float Y , float Angle , Shape * sh ,  Material * m); 
  ~Body();


  void ApplyImpulse(Vec2 impulse , Vec2 contact);;    // Aplikuje impuls
  void ApplyForce(Vec2 force);                       // Aplikuje si��

  void CalcForces(float delta);                      //  Wylicza wp�yw wypadkowej si�y
  void CalcPosition(float delta , bool damping);                    // Wylicza nowe po�o�enie

  void CalcSpeed();

  void translate(Vec2 v);                            // Translacja o wektor 
};


// Po��czenie
class Joint
{
public:
 Body * A;
 Vec2  pA;

 Body * B;
 Vec2  pB;

 float distance;

 Joint();
 Joint(Body * a , Body * b, float d);
 Joint(Body * a , Vec2 PA , Body * b , Vec2 PB , float d);

 void update(float delta);
 void draw();
};

// Struktura kolizji
struct Collision
{
  Body *A;             // Cia�a kt�re koliduj�
  Body *B;

  int contacts_num;    // Liczba kontakt�w

  Vec2 contacts[2];    // Punkty kontaktu ( globalne wsp�rz�dne )

  float penetration;   // Penetracja

  Vec2 normal;         // Normalna 

  float e;             // Mieszana elastyczno�c
  float df;            // Mieszane dynamiczne tarcie
  float sf;            // Mieszane statyczne tarcie

  void Initialize(float delta,Vec2 Gravity);
};



// Silnik Fizyczny
class Phx
{
    public:

	bool gravitation;                               // Grawitacja
	bool friction;                                  // Tarcie
	bool correction;                                // Korekcja pozycji
	bool damping;                                   // Wytracanie pr�dko�ci k�towej

	bool DebugDraw;

	float percent ;                                 // Poziom korekcji
    float treshhold ;

	vector<Body*> BodyList;                         // Lista wszystkich cia� fizycznych
	vector<Body*> DynamicList;                      // Lista dynamicznych cia�
	vector<Body*> StaticList;                       // Lista statycznych cia�

	vector<Joint*> Joints;                          // Lista joint�w

	vector<Collision*> Collisions;                  // Lista kolizji

	vector<Material> Materials;                     // Materia�y

	Vec2 Gravity;                                   // Wektor grawitacji

	 Phx();                                         // Kreator;
    ~Phx();                                         // Destruktor

	void Step(float delta);                         // Wykonaj jeden krok symulacji

	void ApplyGravitation();                        // Zastosuj grawitacje

	void SearchForCollisions();                     // Szukaj kolizji

	void Check( Shape * A , Shape * B );              // Sprawd� czy dwa obiekty koliduj�

	void ResolveCollisions();                       // Rozwi�� wszystkie kolizje

	void Resolve(Collision * c , float delta);                    // Rozwi�� kolizje

	void PositionalCorrection();                    // Dokonaj poprawki pozycji obiekt�w

	void Correct(Collision * c);                    // Popraw obiekty z kolizji

	void MoveBodies(float delta);                   // Porusz cia�a

	void ResolveJoints(float delta);

	void DeleteBodies();


	void AddBody(Body * b);

	void DrawAll();
};



Vec2 Vector(Vec2 A , Vec2 B);


Collision * CircleVsCircle  (Circle * A , Circle * B);    // Testy logiczne kolizji dw�ch cia�
Collision * PolygonVsPolygon(Poly * A , Poly * B);
Collision * PolygonVsCircle (Poly * A , Circle * B);


void ProjectPolygon(Vec2 axis, Poly * polygon,  float & min,  float & max);
float ProjectionDistance(float minA, float maxA, float minB, float maxB);
void ProjectCircle(Vec2 axis, Circle * circle,  float & min,  float & max);

