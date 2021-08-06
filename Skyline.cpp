#include "Skyline.h"



bool Skyline::dominates_respect_to_p(int dominator, int dominated, int p)
{
    for (int i = 0; i < d; i++)
    {
        //Check if they are in the same corner;
        if ( ((D[p][i] < D[dominator][i]) && (D[p][i] > D[dominated][i])) ||
            ((D[p][i] > D[dominator][i]) && (D[p][i] < D[dominated][i])) )
            return false;
        
        if (std::abs(D[dominated][i] - D[p][i]) < std::abs(D[dominator][i] - D[p][i]))
            return false;
    }
    return true;

}

void Skyline::get_border_points(int p)
{
    for (int i = p+1; i < n; i++)
    {
        bool dominated = false;
        for (int j = 0; j < n; j++)
        {
            if ((j == p) || (j==i))
                continue;

            if (dominates_respect_to_p(j, i, p))
            {
                dominated = true;
                break;
            }
        }
        if (!dominated)
        {
            border_points->at(p)->push_back(i);
            border_points->at(i)->push_back(p);
        }
    }
    

}
bool Skyline::same_corner(int p, int op_1, int op_2)
{
    for (int i = 0; i < d; i++)
    {
        if ( !( (D[p][i] > D[op_1][i] && D[p][i] > D[op_2][i]) || 
            (D[p][i] < D[op_1][i] && D[p][i] < D[op_2][i]) ) )
            return false;
    }
    return true;
}

bool Skyline::is_closest(int p, int bp, int bp2, int dim, double closest_val)
{
    if (D[p][dim] < D[bp][dim])
    {
        if (D[bp2][dim] < closest_val && D[bp2][dim] > D[bp][dim])
            return true;
    }
    else
    {
        if (D[bp2][dim] > closest_val && D[bp2][dim] < D[bp][dim])
            return true;
    }
    return false;
}

double Skyline::get_end_point(int p, int bp, int dim)
{

    double closest_val;
    int closest = -1;
    double max_val = 1;
    double min_val = 0;
    if (D[p][dim] < D [bp][dim])
        closest_val = max_val;
    else
        closest_val = min_val;

    for (int z = 0; z < border_points->at(p)->size(); z++)
    {
        int bp2 = border_points->at(p)->at(z);
        if (bp2 == bp)
            continue;

        if (!same_corner(p, bp, bp2))
            continue;

        if (is_closest(p, bp, bp2, dim, closest_val))
        {
            closest_val = D[bp2][dim];
            closest = bp2;
        }
    }
    double end_point;
    if (closest == -1)
        end_point = closest_val;
    else
        end_point = (D[closest][dim] + D[p][dim])/2;
}

Line* Skyline::get_new_line(int p, int bp, int dim, double mid_point, double end_point)
{
    Line* l = new Line;
    l->p = p;
    for (int dim2 = 0; dim2 < d; dim2++)
    {
        if (dim2 == dim)
        {
            double begin;
            double end;
            if (mid_point > end_point) 
            {
                begin = end_point;
                end = mid_point;
            }
            else
            {
                begin = mid_point;
                end = end_point;
            }
            l->min.push_back(begin);
            l->max.push_back(end);
        }
        else
        {
            double dim2_mid_point = (D[p][dim2] + D[bp][dim2])/2;
            l->min.push_back(dim2_mid_point);
            l->max.push_back(dim2_mid_point);
        }
    }
    l->start = NULL;
    l->end = NULL;
    return l;
}

void Skyline::set_dominance_areas()
{
    for (int i = 0; i < n; i++)
    {
        get_border_points(i); 
    }

    double max_val = 1;
    double min_val = 0;
    for (int i = 0; i < ceil(static_cast<double>(n)/l); i++)
    {
        std::map<double, std::vector<Line*> > tmp;
        init_h_lines.push_back(tmp);
        v_lines.push_back(tmp);
        std::map<double, std::set<int> > tmp2;
        init_points.push_back(tmp2);

        std::set<int> s_points;
        for (int jj = 0; jj < l; jj++)
        {
            int p = i*l+jj;
            if (p >= n)
                break;
            std::vector<Line*> p_h_lines;
            std::vector<Line*> p_v_lines;

            bool is_skyline_for_corner = true;
            for (int j = 0; j < border_points->at(p)->size(); j++)
            {
                int bp = border_points->at(p)->at(j);
                bool in_region = false;
                for (int z = 0; z < d; z++)
                {
                    if(D[p][z]>D[bp][z])
                        in_region = true;
                }
                if (!in_region)
                    is_skyline_for_corner = false;

                for (int dim = 0; dim < d; dim++)
                {
                    double mid_point = (D[p][dim] + D[bp][dim])/2;
                    double end_point = get_end_point(p, bp, dim);

                    if (dim == 0)
                    {
                        Line* l = get_new_line(p, bp, dim, mid_point, end_point);
                        p_h_lines.push_back(l);

                        if (l->max[0] == max_val)
                        {
                            std::vector<Line*> h; 
                            h.push_back(l);
                            init_h_lines[i].insert(std::make_pair(l->min.at(1), h));
                        }

                    }
                    else
                    {
                        Line* l = get_new_line(p, bp, dim, mid_point, end_point);
                        std::vector<Line*> v; 
                        v.push_back(l);
                        auto res = v_lines[i].insert(std::make_pair(l->min.at(0), v));
                        if (res.second == false)
                            ((res.first)->second).push_back(l);
                        p_v_lines.push_back(l);
                    }
                }
            }
            if (is_skyline_for_corner)
                s_points.insert(p);

            for (auto it = p_v_lines.begin(); it != p_v_lines.end(); it++)
            {
                Line* v_l = *it;
                for (auto h_it = p_h_lines.begin(); h_it != p_h_lines.end(); h_it++)
                {
                    Line* h_l = *h_it;
                    if ((h_l->min.at(0) == v_l->min.at(0)) || (h_l->max.at(0) == v_l->min.at(0)))
                    {
                        if (h_l->min.at(1) == v_l->min.at(1))
                            v_l->start = h_l;
                        if (h_l->min.at(1) == v_l->max.at(1))
                            v_l->end = h_l;
                    }
                        
                }

            }
        }

        Line* l1 = new Line;
        l1->p = -1;
        l1->start = NULL;
        l1->end = NULL;
        l1->min.push_back(0.0);
        l1->min.push_back(0.0);
        l1->max.push_back(0.0);
        l1->max.push_back(1.0);
        std::vector<Line*> v; 
        v.push_back(l1);
        v_lines[i].insert(std::make_pair(l1->min.at(0), v));

        l1 = new Line;
        l1->p = -1;
        l1->start = NULL;
        l1->end = NULL;
        l1->min.push_back(0.0);
        l1->min.push_back(0.0);
        l1->max.push_back(1.0);
        l1->max.push_back(0.0);
        std::vector<Line*> h; 
        h.push_back(l1);
        init_h_lines[i].insert(std::make_pair(l1->min.at(1), h));

        l1 = new Line;
        l1->p = -1;
        l1->start = NULL;
        l1->end = NULL;
        l1->min.push_back(1.0);
        l1->min.push_back(0.0);
        l1->max.push_back(1.0);
        l1->max.push_back(1.0);
        std::vector<Line*> v2; 
        v2.push_back(l1);
        v_lines[i].insert(std::make_pair(l1->min.at(0), v2));

        l1 = new Line;
        l1->p = -1;
        l1->start = NULL;
        l1->end = NULL;
        l1->min.push_back(0.0);
        l1->min.push_back(1.0);
        l1->max.push_back(1.0);
        l1->max.push_back(1.0);
        std::vector<Line*> h2; 
        h2.push_back(l1);
        init_h_lines[i].insert(std::make_pair(l1->min.at(1), h2));

        init_points[i].insert(std::make_pair(std::prev(std::prev(init_h_lines[i].end()))->first, s_points));

        auto it = init_h_lines[i].rbegin();
        it++;
        auto end_it = init_h_lines[i].rend();
        end_it--;
        for (; it != end_it; it++)
        {
            Line* l1 = it->second[0];
            auto res = s_points.insert(l1->p);
            if (res.second == false)
                s_points.erase(res.first);

            init_points[i].insert(std::make_pair(std::next(it)->first, s_points));
        }
    }
}

void Skyline::get_dependence()
{
    double c_D = 0;
    for (int i = 0; i < n; i++)
    {
        double intersect_count = 0;
        for (int j = 0; j < border_points->at(i)->size(); j++)
        {
            for (int u = 0; u < n; u++)
            {
                auto res = std::find(border_points->at(u)->begin(),  border_points->at(u)->end(), j);
                if (res != border_points->at(u)->end())
                    intersect_count++;
            }
            
        }
        if (intersect_count/border_points->at(i)->size()>c_D)
            c_D = intersect_count/border_points->at(i)->size();
    }
}

long Skyline::check_tree(TreeNode* root, int depth, int max)
{
    if (root->left == NULL)
    {
        if (depth < max)
        {
            return pow(2, (max-depth)+1);
        }
        return 0;
    }
    return check_tree(root->left, depth+1, max)+check_tree(root->right, depth+1, max);
}

void Skyline::build_tree(TreeNode* root, int dim, int depth) 
{   
    bool is_leaf = true;
    for (int z = 0; z < d; z++) 
    {    
        if ((*root->lines)[z].size() > 0)
        {    
            is_leaf = false;
            break;
        }    
    }    
    if (is_leaf)
    {
        no_leaf_nodes++;
        delete root->lines;
        root->lines = NULL;
        return;
    }
    if ((*root->lines)[dim].size() == 0)
    {    
        build_tree(root, (dim+1)%d, depth);
        return;
    }    
    
    TreeNode* left = new TreeNode;
    left->lines = new std::vector<std::vector<Line*> >;
    TreeNode* right = new TreeNode;
    right->lines = new std::vector<std::vector<Line*> >;
    for (int z = 0; z < d; z++) 
    {
        std::vector<Line*> tmp_lines;
        (*left->lines).push_back(tmp_lines);
        (*right->lines).push_back(tmp_lines);
    }
    left->left = NULL;
    left->right = NULL;
    right->left = NULL;
    right->right = NULL;
    
    double m;
    root->left = left;
    root->right = right;
    root->dim = dim; 
    m = (*root->lines)[dim][(*root->lines)[dim].size()/2]->min[dim];
    root->cut = m; 
    
    for (int z = 0; z < d; z++) 
    {    
        for (auto it = (*root->lines)[z].begin(); it!= (*root->lines)[z].end(); ++it)
        {    
            double z1 = (*it)->min[dim];
            double z2 = (*it)->max[dim];
            if (z1 == m && z2 == m)
            {
                delete *it;
                continue;
            }
            else if (z1 >= m)
                (*right->lines)[z].push_back(*it);
            else if (z2 <= m)
                (*left->lines)[z].push_back(*it);
            else 
            {    
                Line* left_line = new Line;
                left_line->p = (*it)->p;
                left_line->min = (*it)->min;
                left_line->max = (*it)->max;
                left_line->max[dim] = m; 
                left_line->end = left_line->start = NULL;
     
                Line* right_line = new Line;
                right_line->p = (*it)->p;
                right_line->min = (*it)->min;
                right_line->min[dim] = m; 
                right_line->max = (*it)->max;
                right_line->end = right_line->start = NULL;
     
                (*right->lines)[z].push_back(right_line);
                (*left->lines)[z].push_back(left_line);

                delete *it;
     
            }    
        }    
    }    
    
    for (int z = 0; z < d; z++)
    {
        (*root->lines)[z].clear();
    }
    delete root->lines;
    root->lines = NULL;
    
    build_tree(root->left, dim, depth+1);
    build_tree(root->right, dim, depth+1);
}

bool compare_lines(Line* l1, Line* l2)
{
    int d = l1->min.size();
    for (int i = 0; i < d; i++)
    {
        if (l1->min[i] == l1->max[i])
            return l1->min[i] < l2->min[i];
    }
    return false;
}

Skyline::Skyline(int n, int d, int k, int m, int l, char alg, std::string data_loc)
{
    no_leaf_nodes = 0;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    srand (0);

    this->k = k;
    this->l = l;
    this->d = d;

    this->n = n;
    this->m = m;

    D = new double*[n];

    // Reading data points
    std::ifstream in;
    in.open(data_loc);
    for (int i = 0; i < n; i++)
    {
        D[i] = new double[d];
        for (int j = 0; j < d; j++)
            in >> D[i][j];
    }
    in.close();


    std::chrono::steady_clock::time_point start_t = std::chrono::steady_clock::now();
    border_points = new std::vector<std::vector<int>* >;
    for (int i = 0; i < n; i++)
        border_points->push_back(new std::vector<int>);

    std::cout << "find domination areas" << std::endl;
    set_dominance_areas();

    std::chrono::steady_clock::time_point end_t = std::chrono::steady_clock::now();
    int dom_areas_time = (std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count());
    std::cout << "found domination areas in " << dom_areas_time << "ms" << std::endl;

    int no_comp = ceil(static_cast<double>(n)/l);
    std::cout << "no components " << ceil(static_cast<double>(n)/l) << ", n: " << n <<", l: " << l << std::endl;

    int agg_time = 0;
    int kd_tree_time = 0;
    for (int i = 0; i < ceil(static_cast<double>(n)/l); i++)
    {
        start_t = std::chrono::steady_clock::now();
        std::cout << "Aggregating " << i+1 <<"th component " <<  std::endl;
        roots.push_back(new TreeNode);
        roots[i]->lines = new std::vector<std::vector<Line*> >;

        for (int z = 0; z < d; z++) 
        {
            std::vector<Line*> tmp_lines;
            (*roots[i]->lines).push_back(tmp_lines);
        }
        if (alg == 'G')
        {
            GreedyAggregator* Agg = new GreedyAggregator( D, d, n, k, m, init_h_lines[i], v_lines[i], init_points[i], roots[i]);
            Agg->get_agg_tiles_v();
            delete Agg;
        }
        else
        {
            DPAggregator* Agg = new DPAggregator( D, d, n, k, m, init_h_lines[i], v_lines[i], init_points[i], roots[i]);
            Agg->get_agg_tiles_v();
            delete Agg;
        }

        for (int z = 0; z < d; z++) 
        {
            std::sort((*roots[i]->lines)[z].begin(), (*roots[i]->lines)[z].end(), compare_lines );
        }
        end_t = std::chrono::steady_clock::now();
        int curr_agg_time = (std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count());
        agg_time+=curr_agg_time;
        std::cout << "aggreattion finished in  " << dom_areas_time << "ms" << std::endl;

        start_t = std::chrono::steady_clock::now();

        std::cout << "Building tree for component "  << i <<  std::endl;
        build_tree(roots[i], 0, 0);


        end_t = std::chrono::steady_clock::now();
        int curr_kd_tree_time = (std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count());
        std::cout << "Built tree in "  << curr_kd_tree_time << "ms" << std::endl;
        kd_tree_time+=curr_kd_tree_time;

    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int total_time = (std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count());
    std::cout << "Total no. leaf nodes "  << no_leaf_nodes << std::endl;
    std::cout << "Total time "  << total_time << "s" << std::endl;
}


Skyline::~Skyline()
{
    for (int i = 0; i < n; i++)
        delete border_points->at(i);
    delete border_points;


    for (int i = 0; i < n; i++)
        delete[] D[i];
    delete[] D;
}
