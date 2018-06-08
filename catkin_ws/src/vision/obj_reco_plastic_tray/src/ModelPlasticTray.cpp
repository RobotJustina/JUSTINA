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

#include "obj_reco_plastic_tray/ModelPlasticTray.h"

 using namespace std;

ModelPlasticTray::ModelPlasticTray(){
	
}


bool ModelPlasticTray::setObject(string name, string nameMethodSegmentation,Scalar minScalarSegmentation,Scalar maxScalarSegmentation, float averageAreaPixels ){
	names.push_back(name);
	namesMethodsSegmentation.push_back(nameMethodSegmentation);
	vector <Scalar> scalars;
	scalars.push_back(minScalarSegmentation);
	scalars.push_back(maxScalarSegmentation);
	min_maxScalarsSegmentation.push_back(scalars);
	averagesAreasPixels.push_back(averageAreaPixels);
	
	
	if(nameMethodSegmentation=="HSV"){
		methodsSegmentation.push_back(Segmenter::colorSegmentHSV);
	}
	else if(nameMethodSegmentation=="HLS"){
		methodsSegmentation.push_back(Segmenter::colorSegmentHLS);
	}
	else if(nameMethodSegmentation=="BGR"){
		methodsSegmentation.push_back(Segmenter::colorSegmentBGR);
	}
	else{
		methodsSegmentation.push_back(Segmenter::colorSegmentHSV);
	}
			

}


bool ModelPlasticTray::loadKnowledgeBase(string path_file){
	loadKnowledgeBase();
}

bool ModelPlasticTray::loadKnowledgeBase(){
	Scalar minScalar;
	Scalar maxScalar;
	

	//plastic_tray
	minScalar = Scalar(65, 42, 110);
	maxScalar = Scalar(81, 95, 167);
	setObject("plastic_tray", "HSV", minScalar,maxScalar);
	
}


int ModelPlasticTray::size(){
	return names.size();
}
