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

#include "obj_reco_dishwasher/ModelDishwasher.h"

 using namespace std;

ModelDishwasher::ModelDishwasher(){
	
}


bool ModelDishwasher::setObject(string name, string nameMethodSegmentation,Scalar minScalarSegmentation,Scalar maxScalarSegmentation, float averageAreaPixels ){
	names.push_back(name);
	namesMethodsSegmentation.push_back(nameMethodSegmentation);
	vector <Scalar> scalars;
	scalars.push_back(minScalarSegmentation);
	scalars.push_back(maxScalarSegmentation);
	min_maxScalarsSegmentation.push_back(scalars);
	averagesAreasPixels.push_back(averageAreaPixels);
	
	
	if(nameMethodSegmentation=="HSV"){
		methodsSegmentation.push_back(SegmenterDishwasher::colorSegmentHSV);
	}
	else if(nameMethodSegmentation=="HLS"){
		methodsSegmentation.push_back(SegmenterDishwasher::colorSegmentHLS);
	}
	else if(nameMethodSegmentation=="BGR"){
		methodsSegmentation.push_back(SegmenterDishwasher::colorSegmentBGR);
	}
	else{
		methodsSegmentation.push_back(SegmenterDishwasher::colorSegmentHSV);
	}
			

}


bool ModelDishwasher::loadKnowledgeBase(string path_file){
	loadKnowledgeBase();
}

bool ModelDishwasher::loadKnowledgeBase(){
	Scalar minScalar;
	Scalar maxScalar;
	

	//dishwasher
	minScalar = Scalar(13, 18, 46);
	maxScalar = Scalar(174, 62, 77);
	setObject("dishwasher", "HSV", minScalar,maxScalar);
	
	//sink
// 	minScalar = Scalar(17, 34, 9);
// 	maxScalar = Scalar(174, 232, 68);
// 	setObject("sink", "HSV", minScalar,maxScalar);
	
	
	
}


int ModelDishwasher::size(){
	return names.size();
}
