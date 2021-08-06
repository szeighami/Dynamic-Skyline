#include "Skyline.h"

int main(int arg_c, char** arg_v)
{

    int n = std::stoi(arg_v[1]);
    int d = std::stoi(arg_v[2]);
    int k = std::stoi(arg_v[3]);
    int m = std::stoi(arg_v[4]);
    int l = std::stoi(arg_v[5]);
    char alg = arg_v[6][0];
    std::string data_loc(arg_v[7]);

    Skyline* S = new Skyline(n, d, k, m, l, alg, data_loc);
    delete S;

    return 0;
}


