#include <stdio.h>
#include <stdint.h>

int main(int argc, char const *argv[]) {
    // uint8_t x = 1;
    // uint8_t y = 2;
    // float z;
    // z = (float) (((int16_t) (x << 8)) | y);
    // printf("%f\n", z);
    // int16_t a1 = 2;
    // printf("%f\n", (float)a1);
    float f = 3.14159;
    char buffer[200];

    sprintf(buffer, "%.5f", f);
    for (int i = 0; buffer[i]; i++) {
        printf("%c", buffer[i]);
    }
    printf("%c", '\n');
}
