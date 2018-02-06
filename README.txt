----------------------
CMSC691 Assignment 1
----------------------
Xiaoxia Zheng / CE83376


----------------------
Command Line Arguments
----------------------
Example: ./main ogre-skeleton.bf pose.dmat output-%05d.pose
	./main -i 30 ogre-skeleton.bf pose.dmat output-%05d.pose	

	"-i nframes": where nframes are interpolated between each pair of poses.
	"ogre-skeleton.bf" & "pose.dmat" are the files that we need to input
	"output-%05d.pose" is the format that we output the files. For this
			 project, you'll have two output file.

-----------------
Project sturcture
-----------------
1. If "-i nframes" is parsed to the program, then start the interpolation part of the program.
   Then output nframes pose files.
   Otherwise, strat the normal program to output two pose files.
2. Create data structure to store input data.
3. Convert every pose joint to matrix.
4. Using recursive function to iterate transforms from root to target joints.
5. Output result to files according to the numbers of poses.

--------------     
Problems I met
--------------
1. Difficult to debug the recursive function.
2. Matrix multiplication has order. I get some help from the Animation text book in this part.
