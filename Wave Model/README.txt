        KU PING CHENG 1001282

====  INSTRUCTIONS  ====

-Using command prompt, go to file directory:
  command ./a3 <integrator> <step>

  where integrator can be  'e' - ForwardEuler, 't' - Trapezoidal,  'r' - RK4 
  step:  can be any decimal number

-Testing results

  For my solution file, by using RK4 method, step size should be smaller than 0.035 for the largest cloth system 
  to perform well. Using Trapezodial method requires a much smaller step size, which is around 0.005. Using Euler
  method often doesn't even provide a stable pendulum system no matter how small the step size is; however, it is 
  still intersting to watch them explode LOL.


-KEYS:

'1' : SimpleSystem
'2' : PendulumSystem 
'3' : ClothSystem, small (8x8 particles)
'4' : ClothSystem, large (20x20 particles)

'w' : toggle different wiring formats (There are 2 formats in Pendulum system and 7 in ClothSystem)
'b' : toggle breeze effect
'r' : reset all systems
' ' : set camera to front view


-Questions

Did you collaborate with anyone in the class? If so, let us know who you talked to and what sort of help you gave or received.
  Yes, I worked with Mu Wenchaun.

Were there any references (books, papers, websites, etc.) that you found particularly helpful for completing your assignment? Please provide a list.
  Lecture slides (especially on steps, vertex mesh construction)

Are there any known problems with your code? If so, please provide a list and, if possible, describe what you think the cause is and how you might fix them if you had more time or motivation. This is very important, as we're much more likely to assign partial credit if you help us understand what's going on.
  Currently no.

Did you do any extra credit? If so, let us know how to use the additional features. If there was a substantial amount of work involved, describe what and how you did it.
  1. Breeze effect simulation (by toggle 'b')
  2. Wireframe & Mesh visualisation (by toggle 'w')

Got any comments about this assignment that you'd like to share? Was it too long? Too hard? were the requirements unclear? Did you have fun, or did you hate it? Did you learn something, or was it a total waste of your time? Feel free to be brutally honest; we promise we won't take it personally.
  One of the most interesting one though! Really had fun working on it!