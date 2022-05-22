#include <algorithm>
#include <stdint.h>

template <typename T1, typename T2>
auto mod(T1 x, T2 y)
{
    return (x / y - (int)(x / y)) * y;
}

template <typename T>
T mean(T array[], uint32_t len)
{
    T sum = 0.0;
    for(uint32_t i=0; i < len; i++)
        sum += array[i];

    return sum / len;
}

template <typename T>
T median(T array[], uint32_t len)
{
    T arr[len];
    std::copy(array, array+len, arr);
    std::sort(arr, arr+len);
    return arr[len / 2];
}
