# rcsmode_reflector
the code contorl robot to reflector goal by pid !
the goal exchange while controling the robot due to the goal is achieved by reflector information .the robot position also changed as run.ido nothave a fixed coordinate .(ros navigation by the fixed global map) so i fixed the goal coordinate and caculate the relative pose between goal and robot under goal coordinate.
under robot coordinate we achieve different goal when the reflector at different side .i get the goal (0,-700) when the reflector is at the side of robot ,while i get the goal (0,700) when the reflector is at left side of the robot.
