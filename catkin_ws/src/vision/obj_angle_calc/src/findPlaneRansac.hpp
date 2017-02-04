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
	//std::cout << "norm_p:  " << cv::norm(point) << std::endl;
	if(point == cv::Point3d(0, 0, 0) )
		isValidPoint = false;

	if(point.x < 0.00005 || point.y < 0.0005 || point.z < 0.0005)
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
			rand_x = rand() % W;
			rand_y = rand() % H;
			std::cout << "Ran_x, y:   " << rand_x << "  " << rand_y << std::endl;
			// Verificamos que el punto tomado de la muestra no sea zero
			validPoint = points.at<cv::Point3d>(rand_x, rand_y);

			if( verifyPoint(validPoint) )
			{
				if (sample.rows > 0)
					// Ciclo para verificar que los puntos no esten repetidos
					// en la muestra
					for (int j = 0; j < sample.rows; j++)
					{
						if( cv::norm(sample.at<cv::Point3d>(j)) == cv::norm(validPoint))
							isReg = true;
					}
			}
			else
				isReg = true;
		}
		while(isReg);

		validPoint = points.at<cv::Point3d>(rand_x, rand_y);
		sample.push_back(validPoint);
	}

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
	int validPoints;
	double error;

	inliers = 0;
	validPoints = 0;
	// Determinamos el plano por 3 puntos
	p0 = sample.at<cv::Point3d>(0);
	p1 = sample.at<cv::Point3d>(1);
	p2 = sample.at<cv::Point3d>(2);

	std::cout << "p_0 " << p0 << std::endl;
	std::cout << "p_1 " << p1 << std::endl;
	std::cout << "p_2 " << p2 << std::endl;
	std::cout << "--------------------------------------" << std::endl;


	plane3D propusePlane(p0, p1, p2);
	planeComp = propusePlane.GetPlaneComp();

	//std::cout << "findPlaneRansac.-> Plane component: " << planeComp << std::endl;


	for (int i = 0; i < points.rows; i++)
		for(int j = 0; j < points.cols; j++)
		{
			// Calculamos la distancia de cada uno de los puntos al plano
			px = points.at<cv::Point3d>(j, i);
			if ( verifyPoint(px))
			{
				error = propusePlane.DistanceToPoint(px, signedDistance);
				//std::cout << "error: " << error << std::endl;
				// Camparamos si la distancia está dentro de la tolerancia
				if (error < threshold)
				{
					// Añadimos el punto[x, y] al Mat consensus
					inliers++;
					consensus.push_back(px);
				}
				validPoints++;
			}
		}

	//std::cout << "inliers: " << inliers;
	//std::cout << "   ValidPoints: " << validPoints;
	//std::cout << "   Porcentaje: " << 100*inliers/validPoints << std::endl;

	return consensus;
}

