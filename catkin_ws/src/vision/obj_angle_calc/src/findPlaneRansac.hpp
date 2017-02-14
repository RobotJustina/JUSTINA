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


	if( std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) )
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
	bool signedDistance = false;
	int W = points.cols;
	int H = points.rows;
	int bestInliers;
	int currentInliers;
	int attemp;
	int validPoints;
	int bestValidPoints;
	float error;

	//cv::Mat points es la nube de puntos del kinect
	cv::Mat rndSample;
	cv::Point3f px;
	cv::Vec4f bestModelPlane;
	cv::Vec4f currentModelPlane;
	cv::Mat consensus(H, W, CV_32FC3, 0.0f);
	cv::Mat bestImage;

	bestInliers = 0;
	validPoints = 0;
	bestValidPoints = 0;
	error = 0.0;
	attemp = 0;

	bestModelPlane = cv::Vec4f(0.0, 0.0, 0.0);

	std::cout << "points_C:   " << points.cols << std::endl;
	std::cout << "points_R:   " << points.rows << std::endl;

	while(attemp < 200)
	{
		//consensus.release();
		currentInliers =0;
		validPoints = 0;
		currentModelPlane = cv::Vec4f(0.0, 0.0, 0.0);

		//Obtenemos una muestra de tres puntos
		rndSample = randomSample(3, points);

		// Determinamos el plano por 3 puntos
		plane3D propusePlane( rndSample.at<cv::Point3f>(0), rndSample.at<cv::Point3f>(1), rndSample.at<cv::Point3f>(2) );
		currentModelPlane = propusePlane.GetPlaneComp();

		//std::cout << "currentPlane:  " << currentModelPlane << std::endl;

		for(int j = 0; j < points.rows; j++)
		{
			for (int i = 0; i < points.cols; i++)
			{
				// Calculamos la distancia de cada uno de los puntos al plano
				px = points.at<cv::Point3f>(j, i);
				if ( verifyPoint(px))
				{
					error = propusePlane.DistanceToPoint(px, signedDistance);
					//std::cout << "error: " << error << std::endl;
					// Camparamos si la distancia está dentro de la tolerancia
					if (error < threshold)
					{
						// Añadimos el punto[x, y] al Mat consensus
						consensus.at<cv::Point3f>(j, i) = points.at<cv::Point3f>(j, i);
						currentInliers++;
					}
					validPoints++;
				}
			}

		}

		if (currentInliers > bestInliers)
		{
			bestModelPlane = currentModelPlane;
			bestValidPoints = validPoints;
			bestInliers = currentInliers;
			bestImage = consensus.clone();
			std::cout << "inliers: " << bestInliers;
			std::cout << "   ValidPoints: " << bestValidPoints;
			std::cout << "   Porcentaje: " << 100*(float)(bestInliers)/(float)(bestValidPoints) << std::endl;
			std::cout << "--------------------------------------" << std::endl;
		}



		attemp++;
	}

	std::cout << "BestModel: " << bestModelPlane << std::endl;
	std::cout << "BestInliers: " << bestInliers;
	std::cout << " - validPoints: " << bestValidPoints;
	std::cout << "   Porcentaje: " << 100*(float)(bestInliers)/(float)(bestValidPoints) << std::endl;
	std::cout << "--------------------------------------" << std::endl;

	return consensus;
}

