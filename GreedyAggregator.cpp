#include "GreedyAggregator.h"



std::map<double, TileData*>::iterator GreedyAggregator::update_tile(bool end_moved, Line* v_line, Line* h_end)
{
    auto found_tile = curr_tiles.lower_bound(h_end->min[1]);
    if (found_tile->first != h_end->min[1])
    {
        TileData* t = new TileData;
        t->h_lines.push_back(h_end);
        t->curr_points = std::prev(found_tile)->second->curr_points;
        t->last_points = std::prev(found_tile)->second->last_points;
        t->H = std::prev(found_tile)->second->H;
        t->min_val = std::prev(found_tile)->second->min_val;
        found_tile = curr_tiles.insert(found_tile, std::make_pair(h_end->min[1], t));
    }
    else
    {
        if (h_end->max[0] <= v_line->min[0])
            found_tile->second->h_lines.push_back(h_end);
    }

    return found_tile;
}



void GreedyAggregator::update_h_lines(bool end_moved, std::vector<Line*> v_lines)
{
    if (end_moved)
    {
        for (auto iter = new_h_lines.begin(); iter!= new_h_lines.end(); iter++)
        {
            auto tile = curr_tiles.find((*iter)->min[1]);
            for (auto it_h_l = tile->second->h_lines.begin(); it_h_l != tile->second->h_lines.end(); it_h_l++)
            {
                if ((*it_h_l)->p == (*iter)->p)
                { 
                    tile->second->h_lines.erase(it_h_l);
                    break;
                }
            }
            if (tile->second->h_lines.size() == 0)
            {
                delete tile->second;
                curr_tiles.erase(tile);
            }
        }
        
    }
    else
    {
        for (auto it = v_lines.begin(); it != v_lines.end(); ++it)
        {
            Line* upper_end = (*it)->end;
            Line* lower_end = (*it)->start;
            if (lower_end != NULL)
                update_tile(end_moved, *it, lower_end);
            if (upper_end != NULL)
                update_tile(end_moved, *it, upper_end);
        }
    }
    if (end_moved)
        new_h_lines.clear();
}

bool GreedyAggregator::update_points(TileData* t, std::vector<Line*> v_lines, bool reset)
{
    if (t->h_lines[0]->min[1] == std::prev(curr_tiles.end())->first)
        return false;

    if (reset)
    {
        double t_loc = t->h_lines[0]->min[1];
        if (t_loc != std::prev(curr_tiles.end())->first)
        {
            t->curr_points = t->last_points;
            t->min_val = t->curr_points.size();
        }
        else
            t->min_val = INT_MAX;
    }
    else
    {
        bool changed = false;
        for (auto it = v_lines.begin(); it != v_lines.end(); ++it)
        {
            Line* upper_end = (*it)->end;
            Line* lower_end = (*it)->start;

            bool above_start = lower_end == NULL || lower_end->min[1] <= t->h_lines[0]->min[1];
            bool below_end = upper_end == NULL || upper_end->min[1] >= t->h_lines[0]->min[1];

            if (!( above_start && below_end ))
                continue;

            if ((*it)->p == -1)
                continue;


            changed = true;
            bool add_p = false;
            if (D[(*it)->p][0] < (*it)->min[0])
                add_p = true;

            if (add_p)
            {
                t->curr_points.insert((*it)->p);
                t->last_points.insert((*it)->p);
            }
            else
                t->last_points.erase((*it)->p);
        }

        if (changed)
        {
            if (t->last_points.size() < t->min_val)
                t->min_val = t->last_points.size();
        }
    }

    return true;
}


int GreedyAggregator::get_agg_tiles_h(double v_start, double v_end, std::vector<Line*> v_lines, bool reset, bool first_time, std::vector<double>* res, int& max_occ)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    int N = curr_tiles.size();
    int h_max = N;
    int line_freq = N;

    auto start_it = std::next(curr_tiles.rbegin());
    res->push_back(1);

    bool new_line = true;
    int curr_cost = 0;
    std::set<int> curr_points;
    int curr_min = 0;
    int total_cost = 0;

    bool first_it = true;
    bool not_valid = false;
    for (; start_it != curr_tiles.rend(); start_it++)
    {
        if (!first_time)
        {
            TileData* prev_t = std::prev(start_it)->second;
            for (auto h_it = prev_t->h_lines.begin(); h_it != prev_t->h_lines.end(); h_it++)
            {
                if ((*h_it)->min[0] > v_start && (*h_it)->min[0] <= v_end)
                    new_h_lines.push_back(*h_it);
            }
            update_points(start_it->second, v_lines, reset);
        }
        if (not_valid)
            continue;

        if (first_it)
        {
            curr_cost = start_it->second->curr_points.size();
            curr_points = start_it->second->curr_points;
            curr_min = start_it->second->min_val;
            if (curr_cost-curr_min>k)
                not_valid = true;
            first_it = false;
            continue;
        }

        int tmp = curr_cost;
        for (auto h_it = std::prev(start_it)->second->h_lines.begin(); h_it != std::prev(start_it)->second->h_lines.end(); h_it++)
            curr_points.insert((*h_it)->p);
        curr_cost = curr_points.size();

        if (start_it->second->min_val < curr_min)
            curr_min = start_it->second->min_val;

        if (curr_cost-curr_min>k)
        {
            res->push_back(std::prev(start_it)->first);
            total_cost += tmp;
            curr_cost = start_it->second->curr_points.size();
            curr_points = start_it->second->curr_points;
            curr_min = start_it->second->min_val;
            if (tmp>max_occ)
                max_occ=tmp;


            if (curr_cost-curr_min>k)
                not_valid = true;
        }
    }
    total_cost += curr_cost;
    res->push_back(0);

    std::chrono::steady_clock::time_point end_t = std::chrono::steady_clock::now();
    time_agg_h+=(std::chrono::duration_cast<std::chrono::milliseconds>(end_t - begin).count());
    if (not_valid)
        return INT_MAX;
    return total_cost;
}

void GreedyAggregator::init_curr_tiles()
{
    int pos = 0;
    for (auto it = init_h_lines.begin(); it != init_h_lines.end(); it++)
    {
        TileData* t = new TileData;

        t->H = INT_MAX;
        t->h_lines = (*it).second;
        if (it != std::prev(init_h_lines.end()))
        {
            t->curr_points = init_points[it->first];
            t->last_points = init_points[it->first];
            t->min_val = init_points[it->first].size();
        }
        else
            t->min_val = INT_MAX;

        curr_tiles.insert(std::make_pair(it->first, t));
        pos++;
    }
}

int GreedyAggregator::get_agg_tiles_v()
{
    int N = v_lines.size();

    int no_h_splits = init_h_lines.size()/(k+1);

    auto start_it = std::next(v_lines.rbegin());
    int curr_cost = 0;
    long total_cost = 0;
    bool first_time = true;
    int start = N-2;
    std::vector<double>* res = new std::vector<double>[N];
    double prev_v_line = v_lines.rbegin()->first;
    Line* l;

    int max_occ = 0;
    for (; start_it != v_lines.rend(); start_it++, start--)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        if (first_time)
        {
            init_curr_tiles();
            curr_cost = get_agg_tiles_h(start_it->first, std::prev(start_it)->first, std::prev(start_it)->second, true, true, &res[start], max_occ);
            first_time = false;
            continue;
        }

        int tmp = curr_cost;
        int tmp_max_occ = max_occ;
        update_h_lines(false, std::prev(start_it)->second);
        curr_cost = get_agg_tiles_h(start_it->first, std::prev(start_it)->first, std::prev(start_it)->second, false, false, &res[start], tmp_max_occ);

        if (curr_cost == INT_MAX || res[start].size()>no_h_splits)
        {
            total_cost+=tmp;
            for (auto it = res[start+1].begin(); it != res[start+1].end(); it++)
            {
                l = new Line;
                l->p = -1;
                l->min.push_back(std::prev(start_it)->first);
                l->min.push_back(*it);
                l->max.push_back(prev_v_line);
                l->max.push_back(*it);
                (*root->lines)[1].push_back(l);
            } 
            l = new Line;
            l->p = -1;
            l->min.push_back(std::prev(start_it)->first);
            l->min.push_back(0);
            l->max.push_back(std::prev(start_it)->first);
            l->max.push_back(1);
            (*root->lines)[0].push_back(l);

            prev_v_line = std::prev(start_it)->first;
            res[start].clear();
            update_h_lines(true, std::prev(start_it)->second);
            curr_cost = get_agg_tiles_h(start_it->first, std::prev(start_it)->first, std::prev(start_it)->second, true, false, &res[start], max_occ);
        }
        if (tmp_max_occ>max_occ)
            max_occ = tmp_max_occ;

            
               
        std::chrono::steady_clock::time_point end_t = std::chrono::steady_clock::now();

    }
    for (auto it = res[0].begin(); it != res[0].end(); it++)
    {
        l = new Line;
        l->p = -1;
        l->min.push_back(0);
        l->min.push_back(*it);
        l->max.push_back(prev_v_line);
        l->max.push_back(*it);
        (*root->lines)[1].push_back(l);
    } 
    total_cost+=curr_cost;

    return total_cost;
}



GreedyAggregator::GreedyAggregator(double** D, int d, int n, int k, int m, std::map<double, std::vector<Line*> > init_h_lines, std::map<double, std::vector<Line*> > v_lines, std::map<double, std::set<int> > init_points, TreeNode* root)
{
    this->D = D;
    this->d = d;
    this->n = n;
    this->k = k;
    this->m = m;
    this->init_h_lines = init_h_lines;
    this->v_lines = v_lines;
    this->init_points = init_points;
    this->root = root;
}


GreedyAggregator::~GreedyAggregator()
{
}
