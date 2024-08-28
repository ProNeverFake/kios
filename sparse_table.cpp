
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <utility>

class ST{
    std::vector<std::vector<int>> VT;

    public:
        ST(const std::vector<int>& v){
            int n = v.size();
            int l1 = std::ceil(std::log2(n)) + 1;

            VT.resize(n, std::vector<int>(l1, 0));

            for (int i = 0; i < n; i++){
                VT[i][0] = v[i];
            }

            for (int j = 1; j < l1; j++){
                int pj = 1 << (j - 1);
                for (int i = 0; i + pj < n; i++){
                    VT[i][j] = std::max(VT[i][j-1], VT[i + pj][j-1]);
                }
            }

        }

        int query(int l, int r){
            int lt = r - l + 1;
            int q = std::floor(std::log2(lt));
            return std::max(VT[l][q], VT[r - (1 << q) + 1][q]);
        }
};

int main(){

}