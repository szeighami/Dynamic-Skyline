#ifndef DPAGGREGATOR_H_
#define DPAGGREGATOR_H_
#include "Skyline.h"

struct Line;
struct TreeNode;
struct TileData;



class DPAggregator
{
public:
    DPAggregator(double** D, int d, int n, int k, int m, std::map<double, std::vector<Line*> > init_h_lines, std::map<double, std::vector<Line*> > v_lines, std::map<double, std::set<int> > init_points, TreeNode* root);
    ~DPAggregator();

    void update_h_lines(bool end_moved, std::vector<Line*> v_lines, double v_start, double v_end);
    long get_agg_tiles_v();
    int get_agg_tiles_h(double v_start, double v_end, std::vector<Line*> v_lines, bool reset, bool first_time, std::vector<double>* res);
    std::map<double, TileData*>::iterator update_tile(bool end_moved, Line* v_line, Line* h_end, double v_start, double v_end);
    void init_curr_tiles(double v_start, double v_end);
    bool update_points(TileData* t, int pos, std::vector<Line*> v_lines, bool reset, double v_start, double v_end);

private:
    double** D;
    int n;
    int d;
    int k;
    int m;
    std::vector<std::vector<int>* >* border_points;
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
