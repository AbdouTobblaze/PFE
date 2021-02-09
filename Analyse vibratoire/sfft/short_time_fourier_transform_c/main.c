#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../signal.h"

#define N           (2048)                // number of fft data points
#define W           (N)                   // Window size
#define H           (N/4)                 // Hope size
#define R           (S_L % W)             // how much zeros to add to make the data divisible by the window size
#define SIG_LEN     (S_L + (W - R))   // the true signal lenght (that is divisible by the window size)
#define N_FRAMES    ((SIG_LEN - N)/H + 1)



int main()
{
    printf("\r\n\t___Program that computes the sfft____\n");
    printf("\r\nN  = %d", N);
    printf("\r\nW = %d", W);
    printf("\r\nH = %d", H);
    printf("\r\nR = %d", R);
    printf("\r\nSIG_LEN = %d",SIG_LEN);
    printf("\r\nN_frames = %d", N_FRAMES);
    // step 1 : padding of the signal
    int i;
    float signal[SIG_LEN] = {0};
    printf("\r\nsig_len = %d", SIG_LEN);
    for (i = 0; i < SIG_LEN; i++)
    {
        signal[i] = sig[i]; // copy the raw samples, the rest are 0
    }
    // step 2 : calculate the number of frames
    printf("\r\nN_frames = %d", N_FRAMES);
    // step 3 : create the hamming window
    float hamming[N] = {0};
    for (i = 0; i < N; i++)
    {
        hamming[i] = 0.54f - 0.46f * cos(2*M_PI*(float)i/(float)N);
    }
    // step 4 : creat sfft containers
    float frame[N_FRAMES][N] = {0};
    // step 5 : compute the sffts
    int start = 0;
    for (i = 0; i < N_FRAMES; i++)
    {
        float y_temp[N] = {0};
        int j = 0;
        for(j = start; j < start + N; j++)  // take a slice of the original signal of length = N
        {
            y_temp[j - start] = signal[j];
        }
        int k = 0;
        for(j = 0; j < N; j++)                  // filter with hamming window
        {
            y_temp[j] *= hamming[j];
        }
        for(j = 0; j < N; j++)
        {
            frame[i][j] = y_temp[j];
        }
        start += H;
        printf("\r\n start = %d", start);
    }
        // Result of the overlap frames
        int j = 0;
            for(i = 0; i < N_FRAMES;i++)
            {
                printf("\r\n\r\nsfft[%d] = ",i);
                for(j = 0; j < N; j++)
                {
                    printf("%f, ", frame[i][j]);
                }
            }

    return 0;
}
