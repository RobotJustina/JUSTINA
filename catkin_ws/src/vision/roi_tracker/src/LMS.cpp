

#include "LMS.hpp"

/* FUNCION ENCARGADA DE ESTIMAR POSICION SIGUIENTE CON MUESTRAS PASADAS*/
        double LMS::Stimate()
        {
            //FACTOR DE PASO DEL GRADIENTE DESCENDENTE
            un=u/(Epsilon+(xn_Last[0]*xn_Last[0]));
            ye=0.0;
            for(n=Nw-1; n>0; n--)
            {
                ye=ye+W[n]*xn_Last[n];
                xn_Last[n]=xn_Last[n-1];
            }
            ye=ye+W[0]*xn_Last[0];
//cout<<ye<<endl;
            return ye;
        }

/** FUNCION ENCARGADA DE ACTUALIZAR LOS W   **/
        void LMS::UpdateW(double xn)
        {
            Error=xn-ye;
           //cout<<Error<<"   ";

            for(n=0; n<Nw; n++)
            {
                W[n]=W[n]+un*xn_Last[n]*Error;
            }
            xn_Last[0]=xn;
            return;
        }

/**CONSTRUCTORES**/
        LMS::LMS()
        {
            Nw=8;
            Epsilon=0.01;
            u=0.1;
            ye=0.0;
            for(int i=0;i<Nw;i++){
                W[i]=0.0;
                xn_Last[i]=0.0;
            }
        }

        LMS::LMS(int SizeW,double pEpsilon,double mu)
        {
            Nw=SizeW;
            Epsilon=pEpsilon;
            u=mu;
            ye=0.0;
               for(int i=0;i<Nw;i++){
                W[i]=0.0;
                xn_Last[i]=0.0;
            }
        }


/**GETs**/
        double LMS::GetError(){

        return Error;
        }

        double LMS::Get_ye(){

        return ye;
        }
