/*
 * Hioka.c
 *
 * Este programa calcula el ángulo de
 * arribo de un señal empleando tres
 * micrófonos en un arreglo triangular
 * equilatero empleando el algoritmo propuesto
 * en hioka2005, el ángulo calculado se
 * envía a través del puerto serial.
 *
 * La operación inicia cuando se recibe un caracter
 * "a" y envia el resultado al obtener un caracter
 * "b".
 *
 *  Created on: 22/03/2017
 *      Author: luis
 *
 *  Probado: 23/03/2017
 *      Tester: Luis
 *
 *
 */

#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "Analog.h"
#include "Serial.h"
#include "fpu_cfft.h"
#include "fpu_vector.h"
#include "math.h"
#include "comp.h"

//#define PHAT                //por si se quieren probar SRP-PHAT
//#define DEBUG               //en caso de querer debuggear el programa mediante el puerto serial

#define	M_PI		3.14159265358979323846	// pi


#define STAGE    9//etapas de la FFT
#define N       (1 << STAGE) //numero de datos (2^STAGE)

float x[N];
float x1[N*2]; // Señal del microfono 1, si el número de etapas es impar el resultado queda aqui
float x2[N*2]; // Señal del microfono 2, si el número de etapas es impar el resultado queda aqui
float x3[N*2]; // Señal del microfono 3, si el número de etapas es impar el resultado queda aqui
float CFFToutBuff[N*2]; // Buffer para la FFT, si el número de etapas es par el resultado queda aqui
float CFFTF32Coef[N];   // Bufer para almacenar los coeficientes de Fourier

float xw1[N*2]; //Buffer para resulado del microfono 1
float xw2[N*2]; //Buffer para resulado del microfono 2
float xw3[N*2]; //Buffer para resulado del microfono 3


CFFT_F32_STRUCT cfft; //Objeto tipo CFFT_F32_STRUCT*/
CFFT_F32_STRUCT_Handle hnd_cfft = &cfft; // Handle del objeto anterior*/

volatile uint16_t cont;
volatile Uint16 Rx;
volatile bool init;
unsigned int doaG=0;

char* itoa[] = {"0\0", "1\0", "2\0", "3\0", "4\0", "5\0", "6\0", "7\0", "8\0", "9\0", "10\0", "11\0", "12\0", "13\0", "14\0", "15\0", "16\0", "17\0", "18\0", "19\0", "20\0", "21\0", "22\0", "23\0", "24\0", "25\0", "26\0", "27\0", "28\0", "29\0", "30\0", "31\0", "32\0", "33\0", "34\0", "35\0",
			"36\0", "37\0", "38\0", "39\0", "40\0", "41\0", "42\0", "43\0", "44\0", "45\0", "46\0", "47\0", "48\0", "49\0", "50\0", "51\0", "52\0", "53\0", "54\0", "55\0", "56\0", "57\0", "58\0", "59\0", "60\0", "61\0", "62\0", "63\0", "64\0", "65\0", "66\0", "67\0", "68\0", "69\0", "70\0", "71\0",
			"72\0", "73\0", "74\0", "75\0", "76\0", "77\0", "78\0", "79\0", "80\0", "81\0", "82\0", "83\0", "84\0", "85\0", "86\0", "87\0", "88\0", "89\0", "90\0", "91\0", "92\0", "93\0", "94\0", "95\0", "96\0", "97\0", "98\0", "99\0", "100\0", "101\0", "102\0", "103\0", "104\0", "105\0", "106\0", "107\0",
			"108\0", "109\0", "110\0", "111\0", "112\0", "113\0", "114\0", "115\0", "116\0", "117\0", "118\0", "119\0", "120\0", "121\0", "122\0", "123\0", "124\0", "125\0", "126\0", "127\0", "128\0", "129\0", "130\0", "131\0", "132\0", "133\0", "134\0", "135\0", "136\0", "137\0", "138\0", "139\0", "140\0", "141\0", "142\0", "143\0",
			"144\0", "145\0", "146\0", "147\0", "148\0", "149\0", "150\0", "151\0", "152\0", "153\0", "154\0", "155\0", "156\0", "157\0", "158\0", "159\0", "160\0", "161\0", "162\0", "163\0", "164\0", "165\0", "166\0", "167\0", "168\0", "169\0", "170\0", "171\0", "172\0", "173\0", "174\0", "175\0", "176\0", "177\0", "178\0", "179\0",
			"180\0", "181\0", "182\0", "183\0", "184\0", "185\0", "186\0", "187\0", "188\0", "189\0", "190\0", "191\0", "192\0", "193\0", "194\0", "195\0", "196\0", "197\0", "198\0", "199\0", "200\0", "201\0", "202\0", "203\0", "204\0", "205\0", "206\0", "207\0", "208\0", "209\0", "210\0", "211\0", "212\0", "213\0", "214\0", "215\0",
			"216\0", "217\0", "218\0", "219\0", "220\0", "221\0", "222\0", "223\0", "224\0", "225\0", "226\0", "227\0", "228\0", "229\0", "230\0", "231\0", "232\0", "233\0", "234\0", "235\0", "236\0", "237\0", "238\0", "239\0", "240\0", "241\0", "242\0", "243\0", "244\0", "245\0", "246\0", "247\0", "248\0", "249\0", "250\0", "251\0",
			"252\0", "253\0", "254\0", "255\0", "256\0", "257\0", "258\0", "259\0", "260\0", "261\0", "262\0", "263\0", "264\0", "265\0", "266\0", "267\0", "268\0", "269\0", "270\0", "271\0", "272\0", "273\0", "274\0", "275\0", "276\0", "277\0", "278\0", "279\0", "280\0", "281\0", "282\0", "283\0", "284\0", "285\0", "286\0", "287\0",
			"288\0", "289\0", "290\0", "291\0", "292\0", "293\0", "294\0", "295\0", "296\0", "297\0", "298\0", "299\0", "300\0", "301\0", "302\0", "303\0", "304\0", "305\0", "306\0", "307\0", "308\0", "309\0", "310\0", "311\0", "312\0", "313\0", "314\0", "315\0", "316\0", "317\0", "318\0", "319\0", "320\0", "321\0", "322\0", "323\0",
			"324\0", "325\0", "326\0", "327\0", "328\0", "329\0", "330\0", "331\0", "332\0", "333\0", "334\0", "335\0", "336\0", "337\0", "338\0", "339\0", "340\0", "341\0", "342\0", "343\0", "344\0", "345\0", "346\0", "347\0", "348\0", "349\0", "350\0", "351\0", "352\0", "353\0", "354\0", "355\0", "356\0", "357\0", "358\0", "359\0"};

extern void ADCB_Process(){
	__asm(" nop");
}

extern void ADCA_Process(void){
	static int i = 0;
	ServiceDog();

	if(cont == 0)
		i = 0;

	if(cont<(N<<1)){
		x1[cont] = (float)(ADC_RESULT_PTR[ADCA]->ADCRESULT2*3.3/4096)-1.5;    // Parte real
		x1[cont+1] = 0.0f;  // Parte imaginiaria
		x[i] = x1[cont];
		x2[cont] = (float)(ADC_RESULT_PTR[ADCA]->ADCRESULT3*3.3/4096)-1.5;    // Parte real
		x2[cont+1] = 0.0f;  // Parte imaginiaria

		x3[cont] = (float)(ADC_RESULT_PTR[ADCA]->ADCRESULT4*3.3/4096)-1.5;    // Parte real
		x3[cont+1] = 0.0f;  // Parte imaginiaria

		cont=cont+2;
		i++;
	}
}

extern void Serial_Process(void){
	Rx = SciaRegs.SCIRXBUF.all; //caracter recibido

	if(Rx == 0x61)
		init = true;
	if(Rx == 0x62){
		Serial_print(itoa[doaG]);	//envia el ángulo
		Serial_print("*\0"); 
	}
	if(Rx == 0x07A)
		Serial_print("Estoy Funcionando\n\0");

}



unsigned int doa_est(float *w1, float *w2, float *w3, const int b);
float energy(float *x);
bool vad(float *x, float Es);

int main(void){

	float Es,E;
	unsigned int i, j;
	unsigned int doa_aux[3];
	int diff[3];

#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif


	/*------------------------------------------------------*/
	/*                   Inicialización                     */
	/*------------------------------------------------------*/
    InitSysCtrl();
    InitSysPll(XTAL_OSC,IMULT_20,FMULT_0,PLLCLK_BY_2);
    EDIS;
    InitGpio();
    InitPieCtrl();

    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    //Configura operación del ADC-A
    ADC_Configure(ADCA,16000);

    //Configura los canales 2,3,4 y 5 del ADC-A
    ADC_Init(ADCA, 2);
    ADC_Init(ADCA, 3);
    ADC_Init(ADCA, 4);

    //La interrupción del ADC-A se da cuando termine el canal 4
    ADC_Int(ADCA, 4);


    hnd_cfft->OutPtr  = CFFToutBuff;  // Apuntador al Buffer de salida
    hnd_cfft->Stages  = STAGE;  // Número de etapas de la FFT
    hnd_cfft->FFTSize = N;    // Tamaño de la FFT

    hnd_cfft->CoefPtr = CFFTF32Coef;  // Auntador a los coeficientes de Fourier
    CFFT_f32_sincostable(hnd_cfft);   // Calcula los factores de Fourier


    //Configura el puerto serial
    Serial_Init();
	Serial_Configure(BR9600);
	Serial_Start();

	ADC_Start(ADCA);   //Inicia la conversión del ADC-A
	cont = 0;

    //calculo de la energía del silencio (que filosófico suena esto)
	EINT;
	Es = 0;
	while(cont<(N<<1));
	ServiceDog();
	cont = 0;
	Es = energy(x);
	while(cont<(N<<1));
	ServiceDog();
	cont = 0;
	Es += energy(x);
	Es = Es/2;
	doaG = 0;

	while(1){

#ifndef DEBUG
		init = false;
		while(!init);
#endif
		E = Es;
		for(j=0;j<3;j++){

			//recibe datos y verifica si es ruido o no
			do{
				ADC_Start(ADCA);
				while(cont<(N<<1));
				cont = 0;    	//Una vez llenos los buffers de datos procedemos a realizar el algoritmo
				ADC_Stop(ADCA); //detiene la adquisición para obtener las FFT
				ServiceDog();
			}while(!vad(x,E));


			//FFT mic 1
			hnd_cfft->InPtr = x1;
			CFFT_f32u(hnd_cfft);
			for(i=0;i<(N<<1);i++){
				xw1[i] = hnd_cfft->CurrentInPtr[i];
			}

			//FFT mic 2
			hnd_cfft->InPtr = x2;
			CFFT_f32u(hnd_cfft);
			for(i=0;i<(N<<1);i++){
				xw2[i] = hnd_cfft->CurrentInPtr[i];
			}

			//FFT mic 3
			hnd_cfft->InPtr = x3;
			CFFT_f32u(hnd_cfft);
			for(i=0;i<(N<<1);i++){
				xw3[i] = hnd_cfft->CurrentInPtr[i];
			}

			ServiceDog();

			doa_aux[j]  = doa_est(xw1,xw2,xw3,30); //50
			//doaG +=doa_aux[j];


			DELAY_US(100000); //retraso de 100ms
			ServiceDog();
			E = 0.8*E;

		}//for j

		diff[0] = doa_aux[0]-doa_aux[1]; //diferencia entre primer y segundo frame
		diff[1] = doa_aux[1]-doa_aux[2]; //diferencia entre segundo y tercer frame
		diff[2] = doa_aux[0]-doa_aux[2]; //diferencia entre primer y tercer frame


		if(diff[0]<0)
			diff[0] = -diff[0];
		if(diff[1]<0)
			diff[1] = -diff[1];
		if(diff[2]<0)
			diff[2] = -diff[2];

		if( diff[0]<=diff[1] && diff[0]<=diff[2] )
			doaG = (doa_aux[0]+doa_aux[1])>>1;
		else if ( diff[1]<=diff[0] && diff[1]<=diff[2] )
			doaG = (doa_aux[1]+doa_aux[2])>>1;
		else
			doaG = (doa_aux[0]+doa_aux[2])>>1;

	}//end while(1)

}//end main


/*****************************************************************
 * Calcula el ángulo de arribo empleando la FFT de las tres      *
 * señales capturadas, en la frecuencia discretra "b", empleando *
 * un formador de has de retraso y suma.                         *
 *                                                               *
 * w1,w2,w3: fft de los micrófonos                               *
 * b: frecuencia discreta para el calculo del DOA                *
 *                                                               *
 * return doa: ángulo de arribo                                  *
 ****************************************************************/
unsigned int doa_est(float *w1, float *w2, float *w3, const int b){

	unsigned int theta;
	int doa=0;
	int bw,B;
	float f;
	float ra;
	float max = -10.0f;
	float P[2],P1[2],P2[2],P3[2],Pa[2];
	float Txy, Tyz, Tzx, Tx2y, Tz2y, Gx2y[2], Gz2y[2];

#ifdef DEBUG
	float angle[360];
#endif

	B = b<<1;//multiplica por 2 por la parte imaginaria
	for(theta=0;theta<360;theta++){
		ServiceDog();
		P[0] = 0;
		P[1] = 0;

		P1[0] = 0;
		P1[1] = 0;

		P2[0] = 0;
		P2[1] = 0;

		P3[0] = 0;
		P3[1] = 0;

		Txy = 0.00044117*sin(0.017453*(theta+120));//120 (negativo?)
		Tyz = 0.00044117*sin(0.017453*(theta));
		Tzx = 0.00044117*sin(0.017453*(theta-120));//-120

		Tx2y = Tyz-Txy;
		Tz2y = Tyz-Tzx;
		ra = 0;
		for(bw=10;bw<B;bw+=2){ //salta de dos en dos por la parte imaginaria //14
			f = 31.25*( (float)(bw>>1)+1.0); //15.625

			Gx2y[0] = cos(-6.2832*f*Tx2y);
			Gx2y[1] = sin(-6.2832*f*Tx2y);

			Gz2y[0] = cos(-6.2832*f*Tz2y);
			Gz2y[1] = sin(-6.2832*f*Tz2y);

			//n=0,m=1
			Pa[0] = w1[bw]*w2[bw]+w1[bw+1]*w2[bw+1]; //correlación (real)
			Pa[1] = w1[bw]*w2[bw+1]-w1[bw+1]*w2[bw]; //correlación (imaginaria)
#ifdef PHAT
			ra = sqrt(Pa[0]*Pa[0]+Pa[1]*Pa[1]);
			Pa[0] = Pa[0]/ra;
			Pa[1] = Pa[1]/ra;
#endif

			P1[0] = Gx2y[0]*Pa[0]-Gx2y[1]*Pa[1];
			P1[1] = Gx2y[0]*Pa[1]+Gx2y[1]*Pa[0];



			//n=0,m=2
			Pa[0] = w3[bw]*w1[bw]+w3[bw+1]*w1[bw+1]; //correlación (real)
			Pa[1] = w3[bw]*w1[bw+1]-w3[bw+1]*w1[bw]; //correlación (imaginaria)
#ifdef PHAT
			ra = sqrt(Pa[0]*Pa[0]+Pa[1]*Pa[1]);
			Pa[0] = Pa[0]/ra;
			Pa[1] = Pa[1]/ra;
#endif

			P2[0] = Gz2y[0]*Pa[0]-Gz2y[1]*Pa[1];
			P2[1] = Gz2y[0]*Pa[1]+Gz2y[1]*Pa[0];


			//n=1,m=2
			Pa[0] = w2[bw]*w3[bw]+w2[bw+1]*w3[bw+1]; //correlación (real)
			Pa[1] = w2[bw]*w3[bw+1]-w2[bw+1]*w3[bw]; //correlación (imaginaria)

#ifdef PHAT
			ra = sqrt(Pa[0]*Pa[0]+Pa[1]*Pa[1]);
			Pa[0] = Pa[0]/ra;
			Pa[1] = Pa[1]/ra;
#endif

			P3[0] = Pa[0];
			P3[1] = Pa[1];

			P[0] = P1[0]+P2[0]+P3[0];
			P[1] = P1[1]+P2[1]+P3[1];

			ra += sqrt(P[0]*P[0]+P[1]*P[1]);

		}//bw

//		ra = P[0]*P[0]+P[1]*P[1];
#ifdef DEBUG
		angle[theta] = ra;
#endif
		if(ra >= max){
			max = ra;
			doa = theta;
		}//if
	}//theta


	//__asm(" ESTOP0");


//correciones por la geometría

	doa = 360 -doa;

	if(doa>=330 || doa<=30)
		doa = 0;

	return (unsigned int)doa;
}

/*****************************************************************
 * Calcula la energía de la señal x, para los primeros 512       *
 * puntos.                                                       *
 *                                                               *
 * x: señal de entrada                                           *
 *                                                               *
 * return E: energía de la señal                                 *
 ****************************************************************/
float energy(float *x){
	int i;
	float E;

	E = 0;

	for(i=0;i<N;i++){
		E += x[i]*x[i];
	}

	return E;
}


/*****************************************************************
 * Determina si existe señal de voz mediante la comparación de   *
 * la energía de la señal con un valor de energía de referencia  *
 *                                                               *
 * x: señal de entrada                                           *
 * Es: energía de referencia                                     *
 *                                                               *
 * return true: la señal es voz                                  *
 *        false: la señal NO es voz                              *
 ****************************************************************/
bool vad(float *x, float Es){
	float E;

	E = energy(x);
	ServiceDog();
	if(E>1.5*Es){ //1.5
			return true;
	}else{
		return false;
	}
}
