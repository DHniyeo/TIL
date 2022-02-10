#include<stdio.h>
#include<stdlib.h>
#include<math.h>

#define F 200
#define T 1/F // sampling period
#define A 5.0 // amplitude
#define N 200 // number of sample depth (data size)
#define H 32	// number of harmonics
#define PI 3.14
int main(){
	FILE* file_out;
	file_out = fopen("out.txt","w");
	
	float polynomial[N];
	
	// signal generation (sum of harmonics)
	for(int i=0; i<N; i++){ // time advance
		int hi = 1; // harmonics 1 Hz
		polynomial[i] = 0;
		for(int j=0; j<H; j++){ // freq
			float signal = (float)(A/hi)*sin(2*PI*hi*T*i);
			polynomial[i] += signal;
			hi = hi + 2;
		}
	}
	
	// plot..
	for(int i=0; i<N; i++){ //time adbance
			fprintf(file_out,"%d\t%.2f\n", i, polynomial[i]);
	}

	// fourier analysis
	// int probe_freq = 2;
	for(int j=0; j<H; j++){
		float freq_component = 0;
		int probe_freq = j;
		for(int i=0; i<N; i++){
			float probe = (float)(A)*sin(2*PI*probe_freq*T*i);
			freq_component += polynomial[i]*probe;
		}
	printf("%d Hz component strength is %.lf\n", probe_freq, freq_component);
   // Don exist probe_freq
	}
	return 0;
}
