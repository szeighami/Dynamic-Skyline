#ifndef SKYLINE_H_
#define SKYLINE_H_

#include <bits/stdc++.h> 
#include <string> 
#include <ctime>
#include <cmath>
#include <math.h>
#include <fstream>
#include <iterator>
#include <cstdlib>
#include <chrono>
#include <vector>
#include "DPAggregator.h"
#include "GreedyAggregator.h"




struct Line
{
    int p;
    std::vector<double> min;
    std::vector<double> max;
    Line* end;
    Line* start;
};
struct TileData
{
    std::vector<Line*> h_lines;
    std::set<int> last_points;
    std::set<int> curr_points;
    int H;
    TileData* arg_H;
    int init_H;
    TileData* arg_init_H;
    int add_cost;
    int min_val;
};
struct TreeNode
{
    std::vector<std::vector<Line*> >* lines;
    int dim;
    double cut;
    TreeNode* left;
    TreeNode* right;
};
/*struct TileData
{
    std::vector<Line*> h_lines;
    std::set<int> last_points;
    std::set<int> curr_points;
//    std::vector<int> H;
    int H;
    TileData* arg_H;
    int init_H;
    TileData* arg_init_H;
    int add_cost;
    int min_val;
    //TileData* next;
    //TileData* prev;
};
*/

class Skyline
{
public:
    Skyline(int n, int d, int k, int m, int l, char alg, std::string data_loc);
    ~Skyline();


    //bool intersects(std::vector<std::pair<double, double> > borders, int p, int bp);
    void set_dominance_areas();
    void get_border_points(int p);
    bool dominates_respect_to_p(int dominator, int dominated, int p);
    //unsigned int get_cost(std::vector<std::pair<double, double> > borders);
    void get_dependence();
    bool same_corner(int p, int op_1, int op_2);
    bool is_closest(int p, int bp, int bp2, int dim, double closest);
    double get_end_point(int p, int bp, int dim);
    Line* get_new_line(int p, int bp, int dim, double mid_point, double end_point);

    void build_tree(TreeNode* root, int dim, int depth);
    long check_tree(TreeNode* root, int depth, int max);


private:
    double** D;
    int n;
    int d;
    int k;
    int m;
    int l;
    std::vector<std::vector<int>* >* border_points;

    std::vector<std::map<double, std::vector<Line*> > > init_h_lines;
    std::vector<std::map<double, std::vector<Line*> > > v_lines;
    std::vector<std::map<double, std::set<int> > > init_points;

    std::vector<TreeNode*> roots;
    int no_leaf_nodes;
    int max_height;
    int no_nodes;
    int total_leaf_height;
    int min_leaf_height;

};

#endif
