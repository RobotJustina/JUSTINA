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
bool validPoint(cv::Point3f point);

cv::Mat randomSample(int n, cv::Mat points);

// Obtenemos los puntos que se ajustan al plano definido por tres puntos
cv::Mat findPlaneConsensus(cv::Mat sample, cv::Mat points, double threshold);

// Obtenemos la ecuacion del plano que mejor se justa a los puntos
//std::vector<double> planeRANSAC(cv::Mat points);



bool verifyPoint(cv::Point3f point)
{
	bool isValidPoint = true;
	//std::cout << "norm_p:  " << cv::norm(point) << std::endl;
	if(point == cv::Point3f(0, 0, 0) )
		isValidPoint = false;

	if(point.x < 0.05 || point.y < 0.05 || point.z < 0.05)
		isValidPoint = false;

	if( std::isnan(point.x) )
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
	cv::Point3f validPoint;

	// Generamos numeros aleatorios
	for (int i = 0; i < n; i++)
	{
		do{
			isReg = false;
			rand_x = rand() % W;
			rand_y = rand() % H;
			// Verificamos que el punto tomado de la muestra no sea zero
			validPoint = points.at<cv::Point3f>(rand_x, rand_y);

			if( verifyPoint(validPoint) )
			{
				if (sample.rows > 0)
					// Ciclo para verificar que los puntos no esten repetidos
					// en la muestra
					for (int j = 0; j < sample.rows; j++)
					{
						if( cv::norm(sample.at<cv::Point3f>(j)) == cv::norm(validPoint))
							isReg = true;
					}
			}
			else
				isReg = true;
		}
		while(isReg);

		validPoint = points.at<cv::Point3f>(rand_x, rand_y);
		sample.push_back(validPoint);
	}

	return sample;
}

cv::Mat findPlaneConsensus(cv::Mat sample, cv::Mat points, double threshold)
{
	//cv::Mat points es la nube de puntos del kinect
	cv::Mat consensus;
	cv::Mat rndSample;
	cv::Point3f px;
	cv::Vec4f bestModelPlane;
	cv::Vec4f currentModelPlane;

	bool signedDistance = false;
	int bestInliers;
	int currentInliers;
	int attemp;
	int pixel_x;
	int pixel_y;
	int validPoints;
	float error;

	bestInliers = 0;
	validPoints = 0;
	error = 0.0;
	attemp = 0;

	bestModelPlane = cv::Vec4f(0.0, 0.0, 0.0);
	consensus = points.clone();

	while(attemp < 200)
	{
		currentInliers =0;
		validPoints = 0;
		currentModelPlane = cv::Vec4f(0.0, 0.0, 0.0);

		//Obtenemos una muestra de tres puntos
		rndSample = randomSample(3, points);

		// Determinamos el plano por 3 puntos
		plane3D propusePlane( rndSample.at<cv::Point3f>(0), rndSample.at<cv::Point3f>(1), rndSample.at<cv::Point3f>(2) );
		currentModelPlane = propusePlane.GetPlaneComp();

		//std::cout << "currentPlane:  " << currentModelPlane << std::endl;

		for (int i = 0; i < consensus.rows; i++)
			for(int j = 0; j < consensus.cols; j++)
			{
				// Calculamos la distancia de cada uno de los puntos al plano
				px = consensus.at<cv::Point3f>(i, j);
				if ( verifyPoint(px))
				{
					error = propusePlane.DistanceToPoint(px, signedDistance);
					//std::cout << "error: " << error << std::endl;
					// Camparamos si la distancia está dentro de la tolerancia
					if (error < threshold)
						// Añadimos el punto[x, y] al Mat consensus
						currentInliers++;
					else
						consensus.at<cv::Point3f>(i, j) = cv::Point3f(0.0, 0.0, 0.0);
					validPoints++;
				}
			}

		if (currentInliers > bestInliers)
		{
			bestModelPlane = currentModelPlane;
			bestInliers = currentInliers;
		}

		//std::cout << "inliers: " << currentInliers;
		//std::cout << "   ValidPoints: " << validPoints;
		//std::cout << "   Porcentaje: " << 100*(float)(currentInliers)/(float)(validPoints) << std::endl;
		//std::cout << "--------------------------------------" << std::endl;

		attemp++;
	}

	std::cout << "BestModel: " << bestModelPlane << std::endl;
	std::cout << "BestInliers: " << bestInliers;
	std::cout << "   Porcentaje: " << 100*(float)(bestInliers)/(float)(validPoints) << std::endl;
	std::cout << "--------------------------------------" << std::endl;

	return consensus;
}

