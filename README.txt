------------------
Forward Kinematics
------------------
There are two parts to this project, (1) designing data structures to store the skeleton hierarchy and pose data and mapping the pose data given in local coordinate frames to world space. (2)interpolate between each pair of poses if the command line switch "-i nframes" is passed to the program, where nframes are interpolated between each pair of poses. 


--------------
Input / Output
--------------
filename		description
ogre-skeleton.bf	.bf "bone forest" file containing the skeleton description. 
			This is a crude file format that stores joint vertex rest positions, corresponding 
			column indices into the weights matrix, and indices of parent joints.
pose.dmat		"dense matrix" containing the poses. Each pose is given as a set of 69 "Euler 
				angles" for 23 3-degree-of-freedom ball-and-socket joints.



----------------------
Command Line Arguments
----------------------
Example: ./main ogre-skeleton.bf pose.dmat output-%05d.pose
	./main -i 30 ogre-skeleton.bf pose.dmat output-%05d.pose	

	"-i nframes": where nframes are interpolated between each pair of poses.
	"ogre-skeleton.bf" & "pose.dmat" are the files that we need to input
	"output-%05d.pose" is the format that we output the files. For this
			 project, you'll have two output file.

