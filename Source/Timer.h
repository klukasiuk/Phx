#pragma once

// Prosty timer
class Timer
{

 private:

  float elapsed;

  float StartTime;

public :

	Timer(void);        // Konstruktor

	void Start();      
	void Stop();
	void Reset();

	float GetTime();     // Zwraca naliczony czas ( milisekundy)
};


class FpsTimer
{
private :

  int freq;

  int frames;

  float StartTime;

  float FPS;

public:

  FpsTimer();
  FpsTimer(int f );

  void update();

  float GetFPS();

};