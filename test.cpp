// 题目：有一个产蛋的养鸡场，每只鸡都有唯一的编号。鸡妈妈可以有若干个孩子，而每只鸡只有一个鸡妈妈（养鸡场中没有公鸡）。给定任意两只鸡的编号，求他们的关系距离。（鸡妈妈和孩子的距离定义为1，同一只鸡妈妈的多个孩子间的距离定义为2，以此类推; 若没有关系，则输出-1）
// 输入为：
// 行数N
// 鸡妈妈1的编号 鸡妈妈1孩子1的编号 … 鸡妈妈1孩子n的编号
// …
// 鸡妈妈N的编号 鸡妈妈N孩子1的编号 … 鸡妈妈1孩子m的编号
// 鸡t1的编号，鸡t2的编号
// 输出为：
// 鸡t1和鸡t2的关系距离

// 输入：
// 2
// 1 2 3
// 4 5 6 7
// 3 5

// 输出：
// -1

#include<iostream>
#include<unordered_set>
#include<unordered_map>
#include<vector>

int main(){

    std::unordered_map<int, std::unordered_set<int>> mp = {{1, {2, 3}}, {4, {5, 6, 7}}};

    int t1 = 2;
    int t2 = 3;

    int result = -1;

    if (mp.find(t1) !=  mp.end() || mp.find(t2) != mp.end()){ // mother found
        if (mp.count(t1) == 0){ 
            std::swap(t1, t2);
        }
        // t1 is a mother
        if (mp[t1].find(t2) == mp[t1].end()) result = -1;
        else result = 1;
    }else{
        for (const auto & ele: mp){
            if (ele.second.count(t1) && ele.second.count(t2)){
                result = 2;
                break;
            }
        }
    }

    std::cout << result << std::endl;
    return 0;
}