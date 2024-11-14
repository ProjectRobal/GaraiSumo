#include "utils.hpp"

void ftoa(float n, char *res, int afterpoint) {
    // Handle negative numbers
    int ipart = (int)n;
    int i = 0;
    if (n < 0) {
        res[i++] = '-';
        n = -n;
        ipart = (int)n;
    }

    // Convert integer part to string
    do {
        res[i++] = ipart % 10 + '0';
        ipart /= 10;
    } while (ipart);

    // Reverse the integer part
    int j = 0, k = i - 1;
    while (j < k) {
        char temp = res[j];
        res[j] = res[k];
        res[k] = temp;
        j++;
        k--;
    }

    // Add decimal point
    res[i++] = '.';

    // Convert fractional part to string
    float fpart = n - (float)ipart;
    for (int j = 0; j < afterpoint; j++) {
        fpart *= 10;
        int digit = (int)fpart;
        res[i++] = digit + '0';
        fpart -= digit;
    }

    res[i] = '\0';
}
