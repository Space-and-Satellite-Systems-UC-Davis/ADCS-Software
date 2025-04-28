#include <stdlib.h>
#include <stdio.h>

#define LEN 256
#define SEARCH 1000000

int main() {
    int bestdiff = 8*LEN;
    int bestseed = -1;

    for (int x = 10000; x<SEARCH; x++) {
    //int x = 84300;
        printf("Searching seed %d\n", x);
        srand(x);
        int results[LEN];

        for (int i = 0; i<LEN; i++) {
            int skip = 0;
            int new = (rand() % 256);

            for (int j = 0; j<i; j++) {
                if (results[j] == new) {
                    i--;
                    skip = 1;
                    break;
                }
            }
            if (!skip) {
                results[i] = new;
                //printf("%8.8b\n", results[i]);
            }
        }

        int sum = 0;
        for (int i = 0; i<LEN - 1; i++)
            sum += results[i];

        results[LEN - 1] = 32640 - sum;
        //printf("%d\n", results[LEN - 1]);
        
        int notflips[8];

        for (int k = 0; k<8; k++) {
            int mask = 1 << k;
            int flip = 0;

            for (int n = 0; n<LEN; n++) {
                if ((results[n+1 % LEN] & mask) == (results[n] & mask)) {
                    flip++;
                }
            }

            //printf("%d\n", flip);
            notflips[k] = flip;
        }

        int diff = 0;

        for (int k = 0; k<8; k++) {
            diff += notflips[k];
        }

        if (diff < bestdiff) {
            bestdiff = diff;
            bestseed = x;
        }
    }

    printf("Best Difference: %d\nBest Seed: %d\n", bestdiff, bestseed);
}
