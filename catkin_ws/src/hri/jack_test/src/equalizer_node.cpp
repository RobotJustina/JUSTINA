#include <iostream>
#include <jack/jack.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "fft.h"

#define NFRAMES 1024

jack_port_t* input_port;
jack_port_t* output_portL;
jack_port_t* output_portR;
jack_client_t* client;

jack_default_audio_sample_t* last_window;
jack_default_audio_sample_t* inter_window;
float* hann_window_values;
float* equalizing_coeff;
complex* last_freq;
complex* inter_freq;
complex* current_freq;

int jack_callback (jack_nframes_t nframes, void *arg)
{
    jack_default_audio_sample_t *in, *outL, *outR;
    
    in = jack_port_get_buffer (input_port, nframes);
    outL = jack_port_get_buffer (output_portL, nframes);
    outR = jack_port_get_buffer (output_portR, nframes);

    //We apply the Hann equation before transforming
    for(int i=0; i < nframes; i++)
    {
        last_freq[i]    = last_window[i] * hann_window_values[i];
        current_freq[i] = in[i] * hann_window_values[i];
    }
    int half_frames = NFRAMES / 2;
    for(int i=0; i < half_frames; i++)
    {
        inter_freq[i] = last_window[i+half_frames] * hann_window_values[i];
        inter_freq[i+half_frames] = in[i] * hann_window_values[i+half_frames];
    }

    //Apply fft to the three windows
    if(!CFFT::Forward(last_freq, nframes) || !CFFT::Forward(inter_freq, nframes) || !CFFT::Forward(current_freq, nframes))
        std::cout << "FFT test node.->Cannot perform fast fourier transform :'(" << std::endl;

    //This is the actual equalizer. We apply the filter to all windows
    for(int i=0; i < nframes; i++)
    {
        last_freq[i] *= equalizing_coeff[i];
        inter_freq[i] *= equalizing_coeff[i];
        current_freq[i] *= equalizing_coeff[i];
    }

    //Inverse transform
    if(!CFFT::Inverse(last_freq, nframes) || !CFFT::Inverse(inter_freq, nframes) || !CFFT::Inverse(current_freq, nframes))
        std::cout << "FFT test node.->Cannot perform inverse fast fourier transform :'(" << std::endl;

    //In the left speaker we put the original signal
    //memcpy (outL, in, nframes * sizeof (jack_default_audio_sample_t));

    //Overlaping of Hann-smoothed windows
    for(int i=0; i < half_frames; i++)
    {
        outR[i] = inter_freq[i].re() + last_freq[i+half_frames].re();
        outR[i+half_frames] = inter_freq[i + half_frames].re() + current_freq[i].re();
    }
    memcpy (outL, outR, nframes * sizeof (jack_default_audio_sample_t));

    //The current window is now the last window
    memcpy (last_window, in, nframes * sizeof (jack_default_audio_sample_t));
    
    return 0;
}

void jack_shutdown (void *arg)
{
	exit (1);
}

void callback_equalizer(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    printf("Equalizer.->Gains: ");
    for(int i=0; i < msg->data.size(); i++)
        printf("%0.3f  ", msg->data[i]);
    std::cout << std::endl;

    equalizing_coeff[0] = msg->data[0];   //Banda de los 31.25
    equalizing_coeff[1] = msg->data[1];   //Banda de los 62.5
    equalizing_coeff[2] = msg->data[2];   //Banda de los 125
    for(int i=3; i < 7; i++)              //Banda de los 250
        equalizing_coeff[i] = msg->data[3];
    for(int i=7; i < 15; i++)             //Banda de los 512 Hz
        equalizing_coeff[i] = msg->data[4];
    for(int i=15; i < 31; i++)            //Banda de los 1024 Hz
        equalizing_coeff[i] = msg->data[5];
    for(int i=31; i < 62; i++)            //Banda de los 2048 Hz
        equalizing_coeff[i] = msg->data[6];
    for(int i=62; i < 124; i++)           //Banda de los 4096 Hz
        equalizing_coeff[i] = msg->data[7];
    for(int i=124; i < 247; i++)          //Banda de los 8192 Hz
        equalizing_coeff[i] = msg->data[8];
    for(int i=247; i < 512; i++)          //Banda de los 16384 Hz
        equalizing_coeff[i] = msg->data[9];

    for(int i=0; i < 512; i++)
        equalizing_coeff[NFRAMES - i - 1] = equalizing_coeff[i];
}

int main (int argc, char *argv[])
{
    std::cout << "INITIALIZING FAST FOURIER TRANSFORM TEST NODE ..." << std::endl;
    ros::init(argc, argv, "equalizer");
    ros::NodeHandle n;
    ros::Subscriber subEq = n.subscribe("/audio/equalizer", 1, callback_equalizer);
    ros::Rate loop(10);
    
	const char *client_name = "jack_fft_test";
	jack_options_t options = JackNoStartServer;
	jack_status_t status;

    last_window  = (jack_default_audio_sample_t*)malloc(NFRAMES*sizeof(jack_default_audio_sample_t));
    inter_window = (jack_default_audio_sample_t*)malloc(NFRAMES*sizeof(jack_default_audio_sample_t));
    hann_window_values = (float*)malloc(NFRAMES*sizeof(float));
    equalizing_coeff = (float*)malloc(NFRAMES*sizeof(float));
    last_freq    = new complex[NFRAMES];
    inter_freq   = new complex[NFRAMES];
    current_freq = new complex[NFRAMES];
    
    for(int i=0; i < NFRAMES; i++)
    {
        last_window[i] = 0;
        inter_window[i] = 0;
        last_freq[i] = 0;
        inter_freq[i] = 0; 
        current_freq[i] = 0;
        hann_window_values[i] = 0.5*(1 - cos(2*M_PI*i/(NFRAMES-1)));
        equalizing_coeff[i] = 1.0;
    }

    client = jack_client_open (client_name, options, &status);
	if (client == NULL)
    {
        std::cout << "jack_client_open() failed, status = " << status << std::endl;
		if (status & JackServerFailed) 
            std::cout << "Unable to connect to JACK server." << std::endl;
		exit (1);
	}
	
	if (status & JackNameNotUnique)
    {
		client_name = jack_get_client_name(client);
        std::cout << "Warning: other agent with the same name is running, " << client_name << " has been assigned to us." << std::endl;
	}
	
	jack_set_process_callback (client, jack_callback, 0);
	jack_on_shutdown (client, jack_shutdown, 0);

    std::cout << "Engine sample rate: " <<  jack_get_sample_rate (client) << std::endl;
	//Create ports
	input_port = jack_port_register (client, "input", JACK_DEFAULT_AUDIO_TYPE,JackPortIsInput, 0);
	output_portL = jack_port_register (client, "outputL",JACK_DEFAULT_AUDIO_TYPE,JackPortIsOutput, 0);
	output_portR = jack_port_register (client, "outputR",JACK_DEFAULT_AUDIO_TYPE,JackPortIsOutput, 0);
	if ((input_port == NULL) || (output_portL == NULL) || (output_portR == NULL))
    {
        std::cout << "Could not create agent ports. Have we reached the maximum amount of JACK agent ports?" << std::endl;
		exit (1);
	}
	if (jack_activate (client))
    {
        std::cout << "Cannot activate client." << std::endl;
		exit (1);
	}
	
    std::cout << "Agent activated. (Y)" << std::endl;
    std::cout << "Connecting ports... " << std::endl;
	 
	const char **serverports_names;
	serverports_names = jack_get_ports (client, NULL, NULL, JackPortIsPhysical|JackPortIsOutput);
	if (serverports_names == NULL)
    {
        std::cout << "No available physical capture (server output) ports." << std::endl;
		exit (1);
	}
	if (jack_connect (client, serverports_names[0], jack_port_name (input_port)))
    {
        std::cout << "Cannot connect input port." << std::endl;
		exit (1);
	}
	free (serverports_names);
	
	serverports_names = jack_get_ports (client, NULL, NULL, JackPortIsPhysical|JackPortIsInput);
	if (serverports_names == NULL)
    {
        std::cout << "No available physical playback (server input) ports." << std::endl;
		exit (1);
	}
	if (jack_connect (client, jack_port_name (output_portL), serverports_names[0]))
    {
        std::cout << "Cannot connect output ports." << std::endl;
		exit (1);
	}
    if (jack_connect (client, jack_port_name (output_portR), serverports_names[1]))
    {
        std::cout << "Cannot connect output ports." << std::endl;
		exit (1);
	}
	free (serverports_names);
	
	
    std::cout << "I think everything is ok (Y)" << std::endl;
	while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
	
	jack_client_close (client);
    std::cout << "Releasing buffer memory..." << std::endl;
    free(last_window);
    free(inter_window);
    free(hann_window_values);
    free(equalizing_coeff);
    delete last_freq;   
    delete inter_freq;  
    delete current_freq;
	exit (0);
}

