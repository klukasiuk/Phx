#include <GLFW/glfw3.h>
#include <iostream>
#include <time.h>
#include <string>
#include <thread>


#include "Mathematic.h"
#include "PHX2.h"
#include "Timer.h"

// Wymiary okienka
const int window_height = 480 ;
const int window_width = 640 ;

bool sim = true;

bool reverse = false;

// Silnik Fizyki
Phx phx;

// Licznik fps
FpsTimer draw(100);
FpsTimer physics(100);

Timer drawtimer;

// Okno
GLFWwindow * window;

// Dwa testowe materia³y
Material test1 = { 0.04f , 0.8f,0.2f,0.1f};
Material test2 = { 0.04f , 0.2f,0.5f,0.4f};
Material comp  = { 0.01f , 0.2f,0.5f,0.4f};


   float PhysAccum = 0.0f ; 
   float PHYS_TIME_STEP = 0.003f; 

   float DrawAccum = 0.0f ; 
   float Draw_TIME_STEP = 0.016f; 


void mouse(GLFWwindow * window, int button , int action , int mods)
{

	if(button == GLFW_MOUSE_BUTTON_LEFT )
    if(action == GLFW_PRESS )
	{
	  double x,y;
	  glfwGetCursorPos(window,&x,&y);

	  y = 480 -y;

			 phx.AddBody(new Body( true,
		                  x,
						  y,
						  0,
						  new Circle(20),
						  &comp
						 )
			   );

	}

	if(button == GLFW_MOUSE_BUTTON_RIGHT )
    if(action == GLFW_PRESS )
	{
	  double x,y;
	  glfwGetCursorPos(window,&x,&y);

	  y = 480 -y;

	 Vec2 tablica[4];

	 tablica[0] = Vec2(20,30);
	 tablica[1] = Vec2(20,-30);
	 tablica[2] = Vec2(-20,-30);
	 tablica[3] = Vec2(-20,30);


	phx.AddBody(new Body( true,
		                  x,
						  y,
						  0,
						  new Poly(tablica,4),
						  &test2
						 )
			   );



	}
}

void button(GLFWwindow * window , int key , int scancode , int action , int mods )
{
	 if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
	  if (key == GLFW_KEY_Z && action == GLFW_PRESS)
	  {
	  PHYS_TIME_STEP +=0.001f;
	  std::cout<<"Fixed Step "<<PHYS_TIME_STEP<<"\n";
	  }
	  if (key == GLFW_KEY_X && action == GLFW_PRESS)
	  {
	  PHYS_TIME_STEP -=0.001f;
	  std::cout<<"Fixed Step "<<PHYS_TIME_STEP<<"\n";
	  }
	  if (key == GLFW_KEY_SPACE  && action == GLFW_PRESS)
	  {
        sim  = !sim;
	  	if(sim)
		std::cout<<"CONTINUE \n";
		else
        std::cout<<"PAUSED \n";
	  }
	 if (key == GLFW_KEY_P && action == GLFW_PRESS)
	  {
        phx.DebugDraw  = !phx.DebugDraw;
	  	if(phx.DebugDraw)
		std::cout<<"DebugDraw ON\n";
		else
        std::cout<<"DebugDraw OFF\n";
	  }

	  if (key == GLFW_KEY_G && action == GLFW_PRESS)
	  {
        phx.gravitation = !phx.gravitation;
	  	if(phx.gravitation)
		std::cout<<"Gravitation ON\n";
		else
        std::cout<<"Gravitation OFF\n";
	  }
	  if (key == GLFW_KEY_D && action == GLFW_PRESS)
	  {
        phx.damping = !phx.damping;
	  	if(phx.damping)
		std::cout<<"Damping ON\n";
		else
        std::cout<<"Damping OFF\n";
	  }

	  if (key == GLFW_KEY_L && action == GLFW_PRESS)
	  {
	  std::cout<<"DrawFps "<<draw.GetFPS()<<"   PhysicsFps "<<physics.GetFPS()<<"\n";
	  }

	  if (key == GLFW_KEY_F && action == GLFW_PRESS)
	  {
        phx.friction = !phx.friction;
	  	if(phx.friction)
		std::cout<<"Friction ON\n";
		else
        std::cout<<"Friction OFF\n";
	  }

	  if (key == GLFW_KEY_N && action == GLFW_PRESS)
	  {
        	 phx.AddBody(new Body( true,
		                  129,
						  400,
						  rand()%360,
						  new Circle(10),
						  &test2
						 )
			   );

			 phx.BodyList[phx.BodyList.size()-1]->velocity = Vec2(500,-500);

		

	  }

	  if (key == GLFW_KEY_Q && action == GLFW_PRESS)
	  {
        reverse = !reverse;
	  } 

	  if (key == GLFW_KEY_U && action == GLFW_PRESS)
	  {
        phx.percent += 0.1f;
		std::cout<<"Postion correction : "<<phx.percent<<"\n";
	  } 

	  if (key == GLFW_KEY_J && action == GLFW_PRESS)
	  {
        phx.percent -= 0.1f;
	    std::cout<<"Postion correction : "<<phx.percent<<"\n";
	  }
	  if (key == GLFW_KEY_I && action == GLFW_PRESS)
	  {
        phx.treshhold += 0.1f;
		std::cout<<"Postion treshhold : "<<phx.treshhold<<"\n";
	  }
	  if (key == GLFW_KEY_K && action == GLFW_PRESS)
	  {
        phx.treshhold -= 0.1f;
	   std::cout<<"Postion treshhold : "<<phx.treshhold<<"\n";
	  }
	  if (key == GLFW_KEY_C && action == GLFW_PRESS)
	  {
        phx.DeleteBodies();
	    std::cout<<"Bodies Cleared\n";
	  }
	  if (key == GLFW_KEY_B && action == GLFW_PRESS)
	  {
        Vec2 tablica[4];

	tablica[0] = Vec2(400,20);
	tablica[1] = Vec2(400,-20);
	tablica[2] = Vec2(-400,-20);
	tablica[3] = Vec2(-400,20);


	phx.AddBody(new Body( false,
		                  300,
						  20,
						  0,
						  new Poly(tablica,4),
						  &test1
						 )
			   );

	phx.AddBody(new Body( false,
		                  300,
						  480,
						  0,
						  new Poly(tablica,4),
						  &test1
						 )
			   );
	
	tablica[0] = Vec2(10,-250);
	tablica[1] = Vec2(10,250);
	tablica[2] = Vec2(-10,250);
	tablica[3] = Vec2(-10,-250);


	phx.AddBody(new Body( false,
		                  12,
						  280,
						  0,
						  new Poly(tablica,4),
						  &test1
						 )
			   );

	




	phx.AddBody(new Body( false,
		                  628,
						  280,
						  0,
						  new Poly(tablica,4),
						  &test1
						 )
			   );



	  }


	  if (key == GLFW_KEY_V && action == GLFW_PRESS)
	  {



		     phx.AddBody(new Body( false,
		                  100,
						  240,
						  0,
						  new Circle(20),
						  &test1
						 )
			   );

		 	 phx.AddBody(new Body( true,
		                  150,
						  240,
						  0,
						  new Circle(20),
						  &test1
						 )
			   );

			 phx.AddBody(new Body( true,
		                  200,
						  240,
						  0,
						  new Circle(20),
						  &test1
						 )
			   );

			 phx.AddBody(new Body( true,
		                  250,
						  240,
						  0,
						  new Circle(20),
						  &test1
						 )
			   );

			 phx.AddBody(new Body( true,
		                  300,
						  240,
						  0,
						  new Circle(20),
						  &test1
						 )
			   );

			 phx.AddBody(new Body( true,
		                  350,
						  240,
						  0,
						  new Circle(20),
						  &test1
						 )
			   );

			 phx.AddBody(new Body( true,
		                  400,
						  240,
						  0,
						  new Circle(20),
						  &test1
						 )
			   );

			 phx.AddBody(new Body( false,
		                  450,
						  240,
						  0,
						  new Circle(20),
						  &test1
						 )
			   );

			 int z = phx.BodyList.size();

			 phx.Joints.push_back ( new Joint(phx.BodyList[z-8],Vec2(10,0),phx.BodyList[z-7],Vec2(-10,0), 30));

			 phx.Joints.push_back ( new Joint(phx.BodyList[z-7],Vec2(10,0),phx.BodyList[z-6],Vec2(-10,0), 30));
		
			 phx.Joints.push_back ( new Joint(phx.BodyList[z-6],Vec2(10,0),phx.BodyList[z-5],Vec2(-10,0), 30));

			 phx.Joints.push_back ( new Joint(phx.BodyList[z-5],Vec2(10,0),phx.BodyList[z-4],Vec2(-10,0), 30));

			 phx.Joints.push_back ( new Joint(phx.BodyList[z-4],Vec2(10,0),phx.BodyList[z-3],Vec2(-10,0), 30));

			 phx.Joints.push_back ( new Joint(phx.BodyList[z-3],Vec2(10,0),phx.BodyList[z-2],Vec2(-10,0), 30));

			 phx.Joints.push_back ( new Joint(phx.BodyList[z-2],Vec2(10,0),phx.BodyList[z-1],Vec2(-10,0), 30));
	  }
}

 void Display()
{
    // kolor t³a - zawartoœæ bufora koloru
    glClearColor( 0.0, 0.0, 0.0, 0.0 );
    
    // czyszczenie bufora koloru
    glClear( GL_COLOR_BUFFER_BIT );
    
    // wybór macierzy modelowania
    glMatrixMode( GL_MODELVIEW );
    
    // macierz modelowania = macierz jednostkowa
    glLoadIdentity();


	glColor3ub(240,240,240);
   
	phx.DrawAll();

	glColor3ub(255,0,0);

	glRecti(0,119,20,121);

	glRecti(0,59,20,61);

	glRecti(0,29,20,31);

	glRectf(0,332  ,20,334);


	glColor3ub(0,255,0);

	glRectf(0,physics.GetFPS() ,10,0);

	if(sim == false)
	{
	glColor3ub(255,0,0);
	glRecti(20,400,80,460);
	}


	glColor3ub(255,0,0);

	glRecti(620,119,640,121);

	glRecti(620,59,640,61);

	glRecti(620,29,640,31);

	glColor3ub(0,255,0);

	glRectf(620,draw.GetFPS() ,630,0);



	// skierowanie poleceñ do wykonania
    glFlush();
    
    // Zamieniam bufory
    glfwSwapBuffers(window);

    // Obs³uguje eventy
    glfwPollEvents();
}

// zmiana wielkoœci okna
 void Reshape()
{
	// obszar renderingu - ca³e okno
    glViewport( 0, 0, window_width, window_height );
    
    // wybór macierzy rzutowania
    glMatrixMode( GL_PROJECTION );
    
    // macierz rzutowania = macierz jednostkowa
    glLoadIdentity();
    
    // rzutowanie prostok¹tne
    glOrtho( 0, window_width, 0, window_height,-3,3 );

	Display();
}

 void Init()
{
    glfwInit();           // Inicjalizuje GLFW

	srand(time(NULL));    // Ziarno dla randa

    window = glfwCreateWindow(window_width, window_height , "CruxPhx", NULL, NULL);  // Tworzê okno

    glfwMakeContextCurrent(window);   // Kontekst

	glfwSetMouseButtonCallback(window,mouse); // Przypinam funkcje obs³ugi myszki

	glfwSetKeyCallback(window,button);  // Przypinam funkcje obs³ugi klawiatury

   // w³¹czenie antyaliasingu linii
    glEnable( GL_LINE_SMOOTH );
        
    // w³¹czenie mieszania kolorów
    glEnable( GL_BLEND );
        
    // równanie mieszania kolorów
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

	Reshape();     // Ustawiam OpenGL
}

int oldmain()
{
	Init();

   float dt = 0.0f; 
   float lastUpdate = glfwGetTime()- 0.016; 


   Display();

   int a = 0;


    while (!glfwWindowShouldClose(window))
    {


		dt = glfwGetTime() - lastUpdate; 
	    lastUpdate = glfwGetTime(); 

		PhysAccum += dt;
		PhysAccum = clamp ( 0.0f , 4*PHYS_TIME_STEP , PhysAccum );

		DrawAccum += dt;
		
		

		Display();
		draw.update();
	
		if(sim)
		{

		while(PhysAccum >= PHYS_TIME_STEP)
		{

        if(! reverse)
		phx.Step(PHYS_TIME_STEP);   
		else
        phx.Step(-PHYS_TIME_STEP);   

		physics.update();
		PhysAccum -= PHYS_TIME_STEP;
		a++;
		}
		}

    }

    glfwTerminate();
    return 0;
}

