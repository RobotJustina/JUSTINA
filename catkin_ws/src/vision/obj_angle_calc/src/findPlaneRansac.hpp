#pragma once
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "plane3D.hpp"

// Muetreo aleatorio de n muestras de la nube de puntos


// Devuele nos valores del punto P[x, y, z] respecto al robot
bool validPoint(cv::Point3d point);

cv::Mat randomSample(int n, cv::Mat points);

// Obtenemos los puntos que se ajustan al plano definido por tres puntos
cv::Mat findPlaneConsensus(cv::Mat sample, cv::Mat points, double threshold);

// Obtenemos la ecuacion del plano que mejor se justa a los puntos
//std::vector<double> planeRANSAC(cv::Mat points);



bool verifyPoint(cv::Point3d point)
{
	bool isValidPoint = true;

	if(point == cv::Point3d(0, 0, 0) || cv::norm(point) > 6.0)
		isValidPoint = false;

	return isValidPoint;
}

//Metodo para tomar aleatoriamente 3 puntos de la nube de puntos
cv::Mat randomSample(int n, cv::Mat points)
{
	int rand_x, rand_y;
	int H = points.rows;
	int W = points.cols;
	//std::cout << "Rows: " << H << std::endl;
	//std::cout << "Cols: " << W << std::endl;
	bool isReg;

	std::vector<int> pixel_i;
	std::vector<int> pixel_j;
	cv::Mat sample;
	cv::Point3d validPoint;

	// Generamos numeros aleatorios
	for (int i = 0; i < n; i++)
	{
		do{
			isReg = false;
			rand_x = rand() % H;
			rand_y = rand() % W;
			// Verificamos que el punto tomado de la muestra no sea zero
			validPoint = points.at<cv::Point3d>(rand_x, rand_y);
			if( !verifyPoint(validPoint) )
				isReg = true;
		}
		while(isReg);
		pixel_i.push_back(rand_x);
		pixel_j.push_back(rand_y);
	}

	for (int i = 0; i < n; i++)
	{
		validPoint = points.at<cv::Point3d>(pixel_i[i], pixel_j[i]);
		sample.push_back(validPoint);
	}
	std::cout << "sample: " << sample << std::endl;
	return sample;
}

cv::Mat findPlaneConsensus(cv::Mat sample, cv::Mat points, double threshold)
{
	//cv::Mat points es la nube de puntos del kinect
	cv::Mat consensus;
	cv::Point3d p0;
	cv::Point3d p1;
	cv::Point3d p2;
	cv::Point3d px;
	cv::Vec4d planeComp;

	bool signedDistance = false;
	int inliers;
	int pixel_x;
	int pixel_y;
	double error;

	inliers = 0;
	// Determinamos el plano por 3 puntos
	p0 = sample.at<cv::Point3d>(0);
	p1 = sample.at<cv::Point3d>(1);
	p2 = sample.at<cv::Point3d>(2);
	std::cout << "p_0 " << p0 << std::endl;
	std::cout << "p_1 " << p1 << std::endl;
	std::cout << "p_2 " << p2 << std::endl;

	plane3D propusePlane(p0, p1, p2);
	//plane3D propusePlane(cv::Point3d(0.0,0.0,0.0), cv::Point3d(0.0,1.0,0.0), cv::Point3d(0.0,0.0,1.0));
	//N(p0, p1, p2);
	//planeComp = propusePlane.GetPlaneComp();

	std::cout << "findPlaneRansac.->Plane component: " << planeComp << std::endl;

/*
	for (int i = 0; i < points.rows; i++)
		for(int j = 0; j < points.cols; j++)
		{
			// Calculamos la distancia de cada uno de los puntos al plano
			px = points.at<cv::Point3d>(j, i);
			if ( verifyPoint(px))
			{
				error = N.DistanceToPoint(px, signedDistance);
				//std::cout << "error: " << error << std::endl;
				// Camparamos si la distancia está dentro de la tolerancia
				if (error < threshold)
					// Añadimos el punto[x, y] al Mat consensus
					inliers++;
					//consensus.push_back(px);
			}
		}
*/
	std::cout << "inliers: " << inliers << std::endl;
	return consensus;
}

