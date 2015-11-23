#pragma once

#include <vector>
#include <iostream>
#include "Mathematic.h"

using std::vector;



// Materia³
struct Material
{
  float density;                // Gêstoœæ
  float restitution;            // Wspó³czynnik odbicia
  float static_friction;        // Tarcie Statyczne
  float dynamic_friction;       // Tarcie dynamiczne
};

// Rodzaje kszta³tów
enum ShapeType
{
  circle,               // Ko³o
  polygon,              // Wielok¹t ( tylko wypuk³y)
  complex               // Z³o¿one cia³o
};

// Prototypy
class Body;
struct Orient;

// Klasa bazowa kszta³tu
class Shape
 {
   public:

   Body * body;                 // WskaŸnik na cia³o posiadaj¹ce ten kszta³t

   bool global;                 // Flaga aktualnoœci danych globalnych

   float radius;                // Promieñ okrêgu opisuj¹cego

   ShapeType type;              // Rodzaj kszta³tu

   bool child;                  // Flaga czy jest dzieckiem innego kszta³tu

   Orient * orient;             // WskaŸnik na orientacje wynikaj¹c¹ z bycia dzieckiem 

   Vec2 center;

   virtual ~Shape(){};
   virtual void  draw() = 0;    // Interfejs
   virtual float area() = 0;
   virtual float inertia() = 0;
   virtual void calc_glob()= 0;
 };



// Ko³o
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

// Wielok¹t ( tylko wypuk³y )
class Poly : public Shape
{
  public:
  Vec2 * vertices;     // tablica lokalnych wierzcho³ków

  Vec2 * GlobVert;     // tablica globalnych wierzcho³ków
  Vec2 * GlobNorm;     // tablica globalnych normalnych

  int size;            // iloœæ wierzcho³ków

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



// Cia³o fizyczne
class Body
{
  public:

  bool dynamic;         // dynamika cia³a

  Vec2 position;        // Pozycja w przestrzeni
  Vec2 velocity;        // Prêdkoœæ

  float angle;          // K¹t obrotu
  float omega;          // Prêdkoœæ k¹towa

  float mass;           // Masa cia³a
  float inv_mass;       // Odwrócona masa

  float inertia;        // Bezw³adnoœæ
  float inv_inertia;    // Odwrócona bezw³adnoœæ

  Vec2 force;           // Si³a
  float torque;         // Moment si³y

  Shape * shape;        // Kszta³t

  Vec2 Imp;
  float epsilon;

  Material * material;  // Materia³

  public:

   Body();
   Body( bool Dynamic , float X , float Y , float Angle , Shape * sh ,  Material * m); 
  ~Body();


  void ApplyImpulse(Vec2 impulse , Vec2 contact);;    // Aplikuje impuls
  void ApplyForce(Vec2 force);                       // Aplikuje si³ê

  void CalcForces(float delta);                      //  Wylicza wp³yw wypadkowej si³y
  void CalcPosition(float delta , bool damping);                    // Wylicza nowe po³o¿enie

  void CalcSpeed();

  void translate(Vec2 v);                            // Translacja o wektor 
};


// Po³¹czenie
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
  Body *A;             // Cia³a które koliduj¹
  Body *B;

  int contacts_num;    // Liczba kontaktów

  Vec2 contacts[2];    // Punkty kontaktu ( globalne wspó³rzêdne )

  float penetration;   // Penetracja

  Vec2 normal;         // Normalna 

  float e;             // Mieszana elastycznoœc
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
	bool damping;                                   // Wytracanie prêdkoœci k¹towej

	bool DebugDraw;

	float percent ;                                 // Poziom korekcji
    float treshhold ;

	vector<Body*> BodyList;                         // Lista wszystkich cia³ fizycznych
	vector<Body*> DynamicList;                      // Lista dynamicznych cia³
	vector<Body*> StaticList;                       // Lista statycznych cia³

	vector<Joint*> Joints;                          // Lista jointów

	vector<Collision*> Collisions;                  // Lista kolizji

	vector<Material> Materials;                     // Materia³y

	Vec2 Gravity;                                   // Wektor grawitacji

	 Phx();                                         // Kreator;
    ~Phx();                                         // Destruktor

	void Step(float delta);                         // Wykonaj jeden krok symulacji

	void ApplyGravitation();                        // Zastosuj grawitacje

	void SearchForCollisions();                     // Szukaj kolizji

	void Check( Shape * A , Shape * B );              // SprawdŸ czy dwa obiekty koliduj¹

	void ResolveCollisions();                       // Rozwi¹¿ wszystkie kolizje

	void Resolve(Collision * c , float delta);                    // Rozwi¹¿ kolizje

	void PositionalCorrection();                    // Dokonaj poprawki pozycji obiektów

	void Correct(Collision * c);                    // Popraw obiekty z kolizji

	void MoveBodies(float delta);                   // Porusz cia³a

	void ResolveJoints(float delta);

	void DeleteBodies();


	void AddBody(Body * b);

	void DrawAll();
};



Vec2 Vector(Vec2 A , Vec2 B);


Collision * CircleVsCircle  (Circle * A , Circle * B);    // Testy logiczne kolizji dwóch cia³
Collision * PolygonVsPolygon(Poly * A , Poly * B);
Collision * PolygonVsCircle (Poly * A , Circle * B);


void ProjectPolygon(Vec2 axis, Poly * polygon,  float & min,  float & max);
float ProjectionDistance(float minA, float maxA, float minB, float maxB);
void ProjectCircle(Vec2 axis, Circle * circle,  float & min,  float & max);

