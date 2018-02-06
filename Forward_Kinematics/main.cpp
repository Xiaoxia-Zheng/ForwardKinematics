//
//  main.cpp
//  project_1
//
//  Created by Xiaoxia Zheng on 9/13/17.
//  Copyright © 2017 Xiaoxia Zheng. All rights reserved.
//

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>


//Define the rotation matrix to trasform euler degree coordinate to matrix.
std::vector<double> rotationMatrix(float radX, float radY, float radZ){
	float cX = cos(radX/180 * M_PI);
	float sX = sin(radX/180 * M_PI);
	
	float cY = cos(radY/180 * M_PI);
	float sY = sin(radY/180 * M_PI);
	
	float cZ = cos(radZ/180 * M_PI);
	float sZ = sin(radZ/180 * M_PI);
	
	std::vector<double> matrix3f=
	{
		cZ*cY, cZ*sY*sX-sZ*cY, cZ*sY*cX+sZ*sX,
		sZ*cY, sZ*sY*sX+cZ*cX, sZ*sY*cX-cZ*sX,
		-sY, cY*sX, cY*cX
	};
	
	//	printf("%f %f %f %f %f %f %f %f %f\n", cZ*cY, cZ*sY*sX-sZ*cY, cZ*sY*cX+sZ*sX,
	//		   sZ*cY, sZ*sY*sX+cZ*cX, sZ*sY*cX-cZ*sX,
	//		   -sY, cY*sX, cY*cX);
	
	return matrix3f;
}


//Using a recursive function to iterate all joints' parents' transforms.
std::vector<double> eulerToWorld(std::vector<std::vector<double> > bf, std::vector<std::vector<double> > pos, std::vector<double> result, int joint_idx){
	int parent_idx;
	std::vector<double> tmp =
	{
		result[0]*pos[joint_idx][0] + result[1]*pos[joint_idx][1] + result[2]*pos[joint_idx][2],
		result[0]*pos[joint_idx][3] + result[1]*pos[joint_idx][4] + result[2]*pos[joint_idx][5],
		result[0]*pos[joint_idx][6] + result[1]*pos[joint_idx][7] + result[2]*pos[joint_idx][8]
	};
	
	//If joint's index equals to 0 means the function has iterate to the root.
	//Then the recursive function stops.
	if (joint_idx == 0) {
		return tmp;
	}else{
		parent_idx = bf[joint_idx][1];
		tmp[0] += bf[parent_idx][2];
		tmp[1] += bf[parent_idx][3];
		tmp[2] += bf[parent_idx][4];
		tmp = eulerToWorld(bf, pos, tmp, parent_idx); //Iterating back to parents'.
		return tmp;
	}
}


//Read the input bf file.
//Store data to new data structure and return a 2D vector.
std::vector<std::vector<double> > read_bf(std::string bf, int rows, int cols){
	std::vector<std::vector<double> >  bf_data(rows, std::vector<double>(cols, 0));
	std::fstream bf_file(bf, std::ios_base::in);
	//std::fstream bf_file("/Users/zhengli/Desktop/project_1/ogre-skeleton.bf", std::ios_base::in);
	
	if (bf_file.is_open()) {
		for (int i =0; i<rows; i++) {
			for (int j=0; j<cols; j++) {
				bf_file >> bf_data[i][j];
				//printf("%f ", bf_data[i][j]);
			}
		}
	}
	bf_file.close();
	return bf_data;
}

//Read the input pose file.
//Store data to new data structure and return a 2D vector.
std::vector<std::vector<double> > read_pos(std::string pos){
	std::fstream pos_file(pos, std::ios_base::in);
	//std::fstream pos_file("/Users/zhengli/Desktop/project_1/pose.dmat", std::ios_base::in);
	
	int pos_cols, pos_rows;
	pos_file>>pos_rows>>pos_cols;
	std::vector<std::vector<double> >  pos_data(pos_rows, std::vector<double>(pos_cols, 0));
	for (int i=0; i<pos_rows; i++) {
		for (int j=0; j<pos_cols; j++) {
			pos_file >> pos_data[i][j];
			//printf("%f ", pos_data[i][j]);
		}
	}
	pos_file.close();
	return pos_data;
}


int main(int argc, const char * argv[]) {
	int bf_cols = 5;
	int bf_rows = 23;
	//argv[1] = "-test";
	//argv[2] = "30";
	std::string bf_input;
	std::string pos_input;
	
	if (argc >= 1) {
		//If the input start with '-i', then interpolate n frames between each pair of poses.
		if (argv[1] == std::string("-i")) {
			std::vector<std::vector<double> > bfData;
			std::vector<std::vector<double> > posData;
			int frames = atoi(argv[2]);
			//printf("%d\n", frames);
			bf_input = argv[3];
			pos_input = argv[4];
			bfData = read_bf(bf_input, bf_rows, bf_cols);
			//printf("\n\n");
			posData = read_pos(pos_input);
			int frame_cout = 0;
			
			for (int pos_num=0; pos_num < frames + 2; pos_num++) {
				std::vector<double> frame_pos;
				for (int i=0; i<posData[0].size(); i++) {
					double tmp = (posData[1][i] - posData[0][i]) * frame_cout / (frames + 1);
					//printf("%f ", tmp);
					frame_pos.push_back(tmp);
					//printf("%f ", frame_pos[i]);
				}
				++frame_cout;
			
				
				//Covert the pos_data to matrix using method rotationMatrix();
				std::vector<std::vector<double> >  pos_matrix;
				for (int j=0; j<frame_pos.size(); j=j+3) {
					std::vector<double> tmp;
					tmp = rotationMatrix(frame_pos[j], frame_pos[j+1], frame_pos[j+2]);
					pos_matrix.push_back(tmp);
				}
				
				//Iterate all parents' transform using recursive function eulerToWorld();
				//Then finally output all joints' world coordinates.
				std::vector< std::vector<double> > output;
				for (int i=0; i<bf_rows; i++) {
					std::vector<double> tmp = {bfData[i][2], bfData[i][3], bfData[i][4]};
					std::vector<double> temp = eulerToWorld(bfData, pos_matrix, tmp, i);
					output.push_back(temp);
					//printf("%f %f %f\n", temp[0], temp[1], temp[2]);
				}
				
				
				//Output final coordinates to file
				std::ofstream ouputFile;
				char file_name[200];
				sprintf(file_name, argv[5], pos_num);
				//sprintf(file_name, "/Users/zhengli/Desktop/project_1/output-%05d.pose", pos_num);
				ouputFile.open(file_name);
				for (int i =0; i<bf_rows; i++) {
					ouputFile << i-1 << ' ' ;
					for (int j=0; j<3; j++) {
						ouputFile << output[i][j] << ' ';
					}
					ouputFile << '\n';
				}
				ouputFile.close();
			}
		}else
		{
			std::vector< std::vector<double> > bfData;
			std::vector< std::vector<double> > posData;
			bf_input = argv[1];
			pos_input = argv[2];
			bfData = read_bf(bf_input, bf_rows, bf_cols);
			posData = read_pos(pos_input);
			
			for (int pos_num=0; pos_num<posData.size(); pos_num++) {
				//Covert the pos_data to matrix using method rotationMatrix();
				std::vector<std::vector<double> >  pos_matrix;
				for (int j=0; j<posData[pos_num].size(); j=j+3) {
					std::vector<double> tmp;
					tmp = rotationMatrix(posData[pos_num][j], posData[pos_num][j+1], posData[pos_num][j+2]);
					pos_matrix.push_back(tmp);
				}
				
				//Iterate all parents' transform using recursive function eulerToWorld();
				//Then finally output all joints' world coordinates.
				std::vector<std::vector<double> > output;
				for (int i=0; i<bf_rows; i++) {
					std::vector<double> tmp = {bfData[i][2], bfData[i][3], bfData[i][4]};
					std::vector<double> temp = eulerToWorld(bfData, pos_matrix, tmp, i);
					output.push_back(temp);
					//printf("%f %f %f\n", temp[0], temp[1], temp[2]);
				}
				
				//Output final coordinates to file
				std::ofstream ouputFile;
				char file_name[200];
				sprintf(file_name, argv[3], pos_num);
				//sprintf(file_name, "/Users/zhengli/Desktop/project_1/output-%05d.pose", pos_num);
				ouputFile.open(file_name);
				for (int i =0; i<bf_rows; i++) {
					ouputFile << i-1 << ' ' ;
					for (int j=0; j<3; j++) {
						ouputFile << output[i][j] << ' ';
					}
					ouputFile << '\n';
				}
				ouputFile.close();
			}
		}
	}
}






