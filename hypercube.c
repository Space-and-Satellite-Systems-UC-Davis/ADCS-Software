#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define NUM 256

//https://en.wikipedia.org/wiki/Gray_code#Converting_to_and_from_Gray_code
int BinaryToGray(int num)
{
    return num ^ (num >> 1); // The operator >> is shift right. The operator ^ is exclusive or.
}

int main() {
    int arr[NUM];
    for (int i = 0; i<NUM; i++) {
        arr[i] = ((i%2==0) ? (BinaryToGray(i/2)) : ((~BinaryToGray((i-1)/2)))) & 0b11111111;
    }

    //printf("char sensor_pair_choice[256] = {");
    for (int i = 0; i<NUM; i++) {
        int diff = arr[i] ^ arr[i+1 % NUM];
        double sum = 0;

        for (int j = 0; j<8; j++)
            sum += (diff & (1 << j)) ? 1 : 0;

        printf("%8.8b %lf\n", arr[i], sqrt(sum));

        for (int j = 0; j<i; j++) {
            if (arr[j] == arr[i]) printf("FAILURE\n");
        }
    }
    //printf("0b%8.8b};", arr[NUM-1]);
}
