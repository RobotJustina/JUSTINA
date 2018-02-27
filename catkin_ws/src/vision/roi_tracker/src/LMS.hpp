#ifndef LMS_HPP_INCLUDED
#define LMS_HPP_INCLUDED

//#include "opencv2/nonfree/features2d.hpp"
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

/** LMS FILTER   **/
    class LMS
    {
    public:
        LMS();
        LMS(int SizeW,double pEpsilon,double mu);

        void UpdateW(double xn);
        double Stimate();
        double GetError();
        double Get_ye();

    private:
        int Nw;
        int n;
        double W[10];
        double xn_Last[10];

        double ye;
        double u;
        double un;
        double Error;
        double Epsilon;
    };
/********/


#endif // LMS_HPP_INCLUDED
