/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2018  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef MODELOBJECTSNOTEXTURENORD_H
#define MODELOBJECTSNOTEXTURENORD_H

#include <string>
#include <vector>
#include <iostream>
#include "opencv2/opencv.hpp"


#include "obj_reco_plastic_tray/Segmenter.h"


using namespace std;
using namespace cv;



class ModelPlasticTray
{
public:
	
	ModelPlasticTray();
	vector< string > names; 
	vector< vector<Scalar >  > min_maxScalarsSegmentation;
	vector<  Mat (*) (Mat,Mat, Scalar, Scalar) > methodsSegmentation;
	vector< float > averagesAreasPixels; 
	
	
	bool setObject(string name, string methodSegmentation,Scalar minScalarSegmentation,Scalar maxScalarSegmentation, float averageAreaPixels = -1.0);
	
	bool loadKnowledgeBase(string path_file);
	
	bool loadKnowledgeBase();
	
	int size();
private:
	vector<string> namesMethodsSegmentation;
	int numberOfModels;
	
	
	
};

#endif // MODELOBJECTSNOTEXTURENORD_H
