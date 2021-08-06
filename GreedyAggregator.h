#ifndef GREEDYAGGREGATOR_H_
#define GREEDYAGGREGATOR_H_
#include "Skyline.h"

struct Line;
struct TreeNode;
struct TileData;



class GreedyAggregator
{
public:
    GreedyAggregator(double** D, int d, int n, int k, int m, std::map<double, std::vector<Line*> > init_h_lines, std::map<double, std::vector<Line*> > v_lines, std::map<double, std::set<int> > init_points, TreeNode* root);
    ~GreedyAggregator();

    void update_h_lines(bool end_moved, std::vector<Line*> v_lines);
    int get_agg_tiles_v();
    int get_agg_tiles_h(double v_start, double v_end, std::vector<Line*> v_lines, bool reset, bool first_time, std::vector<double>* res, int& max_occ);
    std::map<double, TileData*>::iterator update_tile(bool end_moved, Line* v_line, Line* h_end);
    void init_curr_tiles();
    bool update_points(TileData* t, std::vector<Line*> v_lines, bool reset);

private:
    double** D;
    int n;
    int d;
    int k;
    int m;
    TreeNode* root;

    std::map<double, TileData* > curr_tiles;
    std::map<double, std::vector<Line*> > init_h_lines;
    std::map<double, std::vector<Line*> > v_lines;
    std::map<double, std::set<int> > init_points;
    std::vector<Line* > new_h_lines;

    double time_agg_h;
    double time_update_h1;
    double time_update_h21;
    double time_update_h22;
    double time_update_h3;

};
#endif
