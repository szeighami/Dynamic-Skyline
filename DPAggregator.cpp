#include "DPAggregator.h"


std::map<double, TileData*>::iterator DPAggregator::update_tile(bool end_moved, Line* v_line, Line* h_end, double v_start, double v_end)
{
    auto found_tile = curr_tiles.lower_bound(h_end->min[1]);
    if (found_tile->first != h_end->min[1])
    {
        TileData* t = new TileData;
        t->h_lines.push_back(h_end);
        t->curr_points = std::prev(found_tile)->second->curr_points;
        t->last_points = std::prev(found_tile)->second->last_points;
        t->H = std::prev(found_tile)->second->H;
        t->arg_H = std::prev(found_tile)->second->arg_H;
        t->min_val = std::prev(found_tile)->second->min_val;
        found_tile = curr_tiles.insert(found_tile, std::make_pair(h_end->min[1], t));
        new_h_lines.push_back(h_end);
    }
    else
    {
        if (!end_moved)
        {
            if (h_end->max[0] > v_line->min[0])
            {
                found_tile->second->h_lines.push_back(h_end);
                new_h_lines.push_back(h_end);
            }
        }
        else
        {
            if (h_end->max[0] > v_line->min[0])
            {
                for (auto it_h_l = found_tile->second->h_lines.begin(); it_h_l != found_tile->second->h_lines.end(); it_h_l++)
                {
                    if ((*it_h_l)->p == h_end->p)
                    { 
                        found_tile->second->h_lines.erase(it_h_l);
                        break;
                    }
                }
            }
            else
            {
                found_tile->second->h_lines.push_back(h_end);
            }
        }
    }
    TileData* tile = found_tile->second;
    tile->add_cost = 0;
    for (auto it = tile->h_lines.begin(); it != tile->h_lines.end(); ++it)
    {
        Line* h_line = *it;
        bool covers_entirely = h_line->min[0] <= v_start && h_line->max[0] >= v_end;
        bool point_is_added = D[h_line->p][1] > h_line->min[1];
        if (covers_entirely && point_is_added)
            tile->add_cost++;
    }

    return found_tile;
}

void DPAggregator::update_h_lines(bool end_moved, std::vector<Line*> v_lines, double v_start, double v_end)
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

    for (auto it = v_lines.begin(); it != v_lines.end(); ++it)
    {
        Line* upper_end = (*it)->end;
        Line* lower_end = (*it)->start;
        if (lower_end != NULL)
            update_tile(end_moved, *it, lower_end, v_start, v_end);
        if (upper_end != NULL)
            update_tile(end_moved, *it, upper_end, v_start, v_end);
    }
    if (end_moved)
        new_h_lines.clear();
}

bool DPAggregator::update_points(TileData* t, int pos, std::vector<Line*> v_lines, bool reset, double v_start, double v_end)
{
    if (t->h_lines[0]->min[1] == std::prev(curr_tiles.end())->first)
        return false;


    if (reset)
    {
        double t_loc = t->h_lines[0]->min[1];
        if (t_loc != std::prev(curr_tiles.end())->first)
        {
            t->curr_points = init_points[t_loc];
            t->last_points = init_points[t_loc];
            t->min_val = init_points[t_loc].size();
        }
        else
            t->min_val = INT_MAX;
        
        t->add_cost = 0;
        for (auto it = t->h_lines.begin(); it != t->h_lines.end(); ++it)
        {
            Line* h_line = *it;
            if (h_line->p == -1)
                continue;
            bool covers_entirely = h_line->min[0] <= v_start && h_line->max[0] >= v_end;
            bool point_is_added = D[h_line->p][1] > h_line->min[1];
            if (covers_entirely && point_is_added)
                t->add_cost++;
        }
    }

    bool changed = false;
    for (auto it = v_lines.begin(); it != v_lines.end(); ++it)
    {
        Line* upper_end = (*it)->end;
        Line* lower_end = (*it)->start;
        if (!((lower_end == NULL || lower_end->min[1] <= t->h_lines[0]->min[1]) && (upper_end==NULL || upper_end->min[1] >= t->h_lines[0]->min[1])))
            continue;

        if ((*it)->p == -1)
            continue;


        changed = true;
        bool add_p = false;
        if (D[(*it)->p][0] > (*it)->min[0])
            add_p = true;

        if (!reset)
        {
            if (add_p)
            {
                t->curr_points.insert((*it)->p);
                t->last_points.insert((*it)->p);
            }
            else
                t->last_points.erase((*it)->p);
        }
        else
        {
            if (add_p)
            {
                t->last_points.erase((*it)->p);
                t->curr_points.erase((*it)->p);
            }
            else
            {
                t->curr_points.insert((*it)->p);
                t->last_points.insert((*it)->p);
            }
        }

    }

    if (changed)
    {
        if (t->last_points.size() < t->min_val)
            t->min_val = t->last_points.size();
    }

    if (reset)
    {
        if (t->h_lines[0]->min[1] != std::prev(curr_tiles.end())->first)
        {
            init_points[t->h_lines[0]->min[1]] = t->last_points;
            t->curr_points = t->last_points;
            t->min_val = t->curr_points.size();
        }
        else
            t->min_val = INT_MAX;
    }
    return changed;
}

int DPAggregator::get_agg_tiles_h(double v_start, double v_end, std::vector<Line*> v_lines, bool reset, bool first_time, std::vector<double>* res)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    int N = curr_tiles.size();
    int h_max = N;
    int line_freq = this->m;

    auto start_it = curr_tiles.rbegin();
    int prev_split = N-1;
    int start = N-1;
    int total_count = 0;
    curr_tiles.rbegin()->second->H = 0;
    curr_tiles.rbegin()->second->init_H = 0;
    curr_tiles.rbegin()->second->arg_H = NULL;
    curr_tiles.rbegin()->second->arg_init_H = NULL;

    int init_points_pos = start+1;
    bool changed = first_time;
    bool try_next_start = true;
    for (; start_it != curr_tiles.rend(); start_it++, start--)
    {
        if (reset)
        {
            start_it->second->H = start_it->second->init_H;
            start_it->second->arg_H = start_it->second->arg_init_H;
        }
        init_points_pos--;
        if (start_it->second->h_lines.size() == 0)
        {
            init_h_lines.erase(start_it->first);
            init_points.erase(start_it->first);
            delete start_it->second;
            std::reverse_iterator<std::map<double, TileData*>::iterator > rev_it(curr_tiles.erase(std::next(start_it).base()));
            start_it = rev_it;
            start_it--;
            continue;
        }
        if (!first_time)
            changed = update_points(start_it->second, init_points_pos, v_lines, reset, v_start, v_end) || changed;

        if (try_next_start == false)
        {

            start_it->second->H = INT_MAX;
            start_it->second->arg_H = NULL;

            if (reset)
            {
                start_it->second->init_H = start_it->second->H;
                start_it->second->arg_init_H = start_it->second->arg_H;
            }
            continue;
        }
        bool try_next_split = true;
        unsigned int min = INT_MAX;
        int arg_min = -1;
        TileData* arg_min_p = NULL;

        auto split_it = start_it.base();
        int split = start+1;
        int curr_cost = start_it->second->curr_points.size();
        std::set<int> curr_points = start_it->second->curr_points;
        int min_cost = start_it->second->min_val;

        if (start == N - 1)
        {
            min = 0;
            arg_min = N-1;
            arg_min_p = NULL;
        }
        else
        {
            if (curr_cost > min_cost+this->k) 
            { 
                curr_cost = INT_MAX;
                try_next_split = false;
                min = INT_MAX;
            }
            else
            {
                min = curr_cost+split_it->second->H;
                arg_min = split;
                arg_min_p = split_it->second;
            }
        }
        split_it++;
        split++;

        for (; split <= prev_split && split < curr_tiles.size() && split-start<=h_max; split_it++, split++)
        {
            if (try_next_split == false)
                break;

            TileData* tile = std::prev(split_it)->second;
            for (auto h_it = std::prev(split_it)->second->h_lines.begin(); h_it != std::prev(split_it)->second->h_lines.end(); h_it++)
                curr_points.insert((*h_it)->p);
            curr_cost = curr_points.size();
            if (tile->min_val < min_cost)
                min_cost = tile->min_val;


            if (curr_cost > min_cost+this->k) 
            { 
                curr_cost = INT_MAX;
                try_next_split = false;
            }


            int cost = curr_cost + split_it->second->H;
            if (cost < min)
            {
                min = cost;
                arg_min = split;
                arg_min_p = split_it->second;
            }

        }
        if (min == INT_MAX)
            try_next_start = false;

        if (start!=N-1)
        {
            start_it->second->H = min;
            start_it->second->arg_H = arg_min_p;

            if (start % line_freq == 0)
                prev_split = start+1;
            else
                prev_split = arg_min;

        }
        if (reset)
        {
            start_it->second->init_H = start_it->second->H;
            start_it->second->arg_init_H = start_it->second->arg_H;
        }

    }

    res->push_back(0);
    for (TileData* it_t = curr_tiles.begin()->second->arg_H; it_t != NULL; it_t = it_t->arg_H)
        res->push_back(it_t->h_lines[0]->min[1]);

    std::chrono::steady_clock::time_point end_t = std::chrono::steady_clock::now();
    time_agg_h+=(std::chrono::duration_cast<std::chrono::milliseconds>(end_t - begin).count());
    return curr_tiles.begin()->second->H;
}

void DPAggregator::init_curr_tiles(double v_start, double v_end)
{
    int pos = 0;
    for (auto it = init_h_lines.begin(); it != init_h_lines.end(); it++)
    {
        TileData* t = new TileData;

        t->H = INT_MAX;
        t->arg_H = NULL;
        t->h_lines = (*it).second;
        if (it != std::prev(init_h_lines.end()))
        {
            t->curr_points = init_points[it->first];
            t->last_points = init_points[it->first];
            t->min_val = init_points[it->first].size();
        }
        else
            t->min_val = INT_MAX;

        t->add_cost = 0;
        for (auto it = t->h_lines.begin(); it != t->h_lines.end(); ++it)
        {
            Line* h_line = *it;
            if (h_line->p == -1)
                continue;
            bool covers_entirely = h_line->min[0] <= v_start && h_line->max[0] >= v_end;
            bool point_is_added = D[h_line->p][1] > h_line->min[1];
            if (covers_entirely && point_is_added)
                t->add_cost++;
        }
        curr_tiles.insert(std::make_pair(it->first, t));
        pos++;
    }
}

long DPAggregator::get_agg_tiles_v()
{
    int N = v_lines.size();
    int v_max = N;
    int line_freq = this->m;

    std::vector<double>* val_V = new std::vector<double>[N];
    long* V = new long[N];
    int* arg_V = new int[N];
    for (long i = 0; i < N; i++)
    {
        V[i] = INT_MAX;
        arg_V[i] = -1;
    }

    double* line_vals = new double[N];
    int iter2 = 0;
    for (auto iter = v_lines.begin(); iter!= v_lines.end(); iter++, iter2++)
        line_vals[iter2] = iter->first;

    V[N-1] = 0;
    arg_V[N-1] = -1;
    int start = N-2;
    int prev_split = N-1;
    for (auto start_it = std::next(v_lines.rbegin()); start_it != v_lines.rend(); start_it++, start--)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        bool try_next = true;
        unsigned int min = INT_MAX;
        int arg_min = -1;

        auto split_it = start_it.base();
        int split = start+1;
        if (start == N-2)
            init_curr_tiles(start_it->first, split_it->first);
        else
            update_h_lines(true, std::prev(start_it)->second, start_it->first, split_it->first);


        std::vector<std::vector<double> > results;
        for (; split <= prev_split && (split-start <= v_max); split_it++, split++)
        {
            if (try_next == false)
                break;

            std::vector<double> tmp;
            results.push_back(tmp);
            unsigned int c_cost;
            if (start != split - 1)
            {
                update_h_lines(false, std::prev(split_it)->second, start_it->first, split_it->first);
                c_cost = get_agg_tiles_h(start_it->first, split_it->first, std::prev(split_it)->second, false, false, &results[results.size() -1]);
            }
            else
                c_cost = get_agg_tiles_h(start_it->first, split_it->first, std::prev(start_it)->second, true, start == N-2, &results[results.size() -1]);


            if (c_cost == INT_MAX)
                try_next = false;


            double r_cost = V[split];
            if (c_cost + r_cost < min)
            {
                min = c_cost+r_cost;
                arg_min = split;
            }
            
        }
        if (start % line_freq == 0)
            prev_split = start+1;
        else
            prev_split = arg_min;
               
        V[start] = min;
        arg_V[start] = arg_min;
        val_V[start] = results[arg_min - start-1];

        std::chrono::steady_clock::time_point end_t = std::chrono::steady_clock::now();
        time_agg_h = 0;
        time_update_h1 = 0;
        time_update_h21 = 0;
        time_update_h22 = 0;
        time_update_h3 = 0;

    }

    for (int i = 0; i != -1; i = arg_V[i])
    {
        Line* l = new Line;
		l->p = -1;
		l->min.push_back(line_vals[i]);
		l->min.push_back(0);
		l->max.push_back(line_vals[i]);
		l->max.push_back(1);
		(*root->lines)[0].push_back(l);

		for (auto it = val_V[i].begin(); arg_V[i] != -1 && it != val_V[i].end(); it++)
		{
			l = new Line;
			l->p = -1;
			l->min.push_back(line_vals[i]);
			l->min.push_back(*it);
			l->max.push_back(line_vals[arg_V[i]]);
			l->max.push_back(*it);
			(*root->lines)[1].push_back(l);
		} 

    }

    long min = V[0];
    delete[] V;

    return min;
}


DPAggregator::DPAggregator(double** D, int d, int n, int k, int m, std::map<double, std::vector<Line*> > init_h_lines, std::map<double, std::vector<Line*> > v_lines, std::map<double, std::set<int> > init_points, TreeNode* root)
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


DPAggregator::~DPAggregator()
{
}
