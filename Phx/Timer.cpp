#include "Timer.h"
#include <GLFW/glfw3.h>
#include <iostream>


Timer::Timer(void)
{
  elapsed = 0;
  StartTime = 0;
}


void Timer::Start()
{
  StartTime = glfwGetTime();
}

void Timer::Stop()
{
 elapsed += glfwGetTime() - StartTime;
}

void Timer::Reset()
{
 elapsed = 0;
 StartTime = 0;
}

float Timer::GetTime()
{
  return glfwGetTime() - StartTime + elapsed;
}


FpsTimer::FpsTimer()
{
 freq = 100;
 frames = 0;
 StartTime = glfwGetTime();
 FPS = 0;
}

FpsTimer::FpsTimer(int f)
{
 freq = f;
 frames = 0;
 StartTime = glfwGetTime();
 FPS = 0;
}

void FpsTimer::update()
{
 frames++;

 if( frames == freq )
 {
  FPS = frames/( glfwGetTime() - StartTime ) ;

  StartTime = glfwGetTime();
  frames = 0;
 }
}

float FpsTimer::GetFPS()
{
 return FPS;
}