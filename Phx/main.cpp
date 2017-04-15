#include "SimpleGL.h"

#include "Mathematic.h"
#include "PHX2.h"
#include "Timer.h"

#include <string>

using std::string;
using std::to_string;

// Global state flags
bool run;
bool simulation;

// Window dimmensions
const int window_width = 640;
const int window_height = 480;

// Physics engine
Phx phx;

// Test materials
Material test1 = { 0.04f , 0.8f,0.2f,0.1f };
Material test2 = { 0.04f , 0.2f,0.5f,0.4f };
Material comp  = { 0.01f , 0.2f,0.5f,0.4f };

// Simulation parameters
const int sleep_time = 10;

void createSandbox()
{
	Vec2 tab[4];

	tab[0] = Vec2(400, 20);
	tab[1] = Vec2(400, -20);
	tab[2] = Vec2(-400, -20);
	tab[3] = Vec2(-400, 20);

	phx.AddBody(new Body(false,300,20,0,new Poly(tab, 4),&test1));
	phx.AddBody(new Body(false,300,480,0,new Poly(tab, 4),&test1));

	tab[0] = Vec2(10, -250);
	tab[1] = Vec2(10, 250);
	tab[2] = Vec2(-10, 250);
	tab[3] = Vec2(-10, -250);

	phx.AddBody(new Body(false,12,280,0,new Poly(tab, 4),&test1));
	phx.AddBody(new Body(false,628,280,0,new Poly(tab, 4),&test1));
}

// Function called every time key was pressed
void myKeyboardCallback(KeyboardKey key, InputAction action)
{
	if (key == Key_Space && action == Pressed)
		simulation = !simulation;
}

// Function called every time mouse button was pressed
void myMouseCallback(int x, int y, MouseButton button, InputAction action)
{
	y = window_height - y;

	if (button == Mouse_Left && action == Pressed)
	phx.AddBody(new Body(true,x,y,0,new Circle(20),&comp));

	if (button == Mouse_Right && action == Pressed)
	{
		Vec2 tab[4];

		tab[0] = Vec2(20, 30);
		tab[1] = Vec2(20, -30);
		tab[2] = Vec2(-20, -30);
		tab[3] = Vec2(-20, 30);

		phx.AddBody(new Body(true,x,y,0,new Poly(tab, 4),&test2));
	}
}

// Initialization function
void init()
{
	// Setting hint for library to use double buffering ( swapping buffers will be needed)
	setDoubleBuffered(true);

	// Initialization of graphics window with desired size of window
	initGL(640, 480);

	// Setting color used when we clear window
	setClearColor(0, 0, 0);

	// We must register our input callbacks if we want them work
	setKeyboardCallback(myKeyboardCallback);
	setMouseCallback(myMouseCallback);

	// Now global state flags
	run = true;
	simulation = true;

	createSandbox();
}

// Closing function
void release()
{
	// Waiting for users input
	wait();

	// Releasing resources and closing window
	end();
}

// Getting input and input handling
void input()
{
	// Checking for new events and inputs
	checkEvents();
}

// Updating program state
void update()
{
	if(simulation)
	phx.Step(0.01);
	
	sleep(sleep_time);
}

// This function is drawing everything and swaping buffers if there is double buffering
void draw()
{
	// Drawing physics objects
	phx.DrawAll();

	// Draw simulation time
	float time = getTime() / 1000.0f;
	string timeString = "Time elapsed = " + to_string(time).substr(0,5);
	text(10, 10, (char*)timeString.c_str());

	// If simulation is stopped draw red circle in corner
	if (simulation == false)
	{
		setColor(Colors::red);
		circle (window_width - 20, window_height - 20, 15);
	}

	// Swap buffers to show what was drawed
	swap();
}

// Main loop is really simple and self explaining 
int main()
{
	init();

	while(run)
	{
		input();

		update();

		draw();
	}

	release();
}