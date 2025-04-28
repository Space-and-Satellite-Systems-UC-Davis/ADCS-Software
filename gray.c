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
        arr[i] = (i%2==0) ? (BinaryToGray(i)) : ((~BinaryToGray(i)) & 0b11111111);
    }

    printf("char sensor_pair_choice[256] = {");
    for (int i = 0; i<NUM-1; i++) {
        printf("0b%8.8b,", arr[i]);
    }
    printf("0b%8.8b};", arr[NUM-1]);
}
