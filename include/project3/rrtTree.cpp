#include "rrtTree.h"

rrtTree::rrtTree(){
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->cost = 0;	// For RRT*
    root->rand = x_init;
}

rrtTree::~rrtTree(){
    for (int i = 1; i <= count; i++) {
        delete ptrTable[i];
    }
}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void rrtTree::visualizeTree(){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map_original, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(x_init.y/res + map_origin_y)), (int)(Res*(x_init.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(x_goal.y/res + map_origin_y)), (int)(Res*(x_goal.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
        x1 = cv::Point((int)(Res*(ptrTable[i]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[i]->location.x/res + map_origin_x)));
        x2 = cv::Point((int)(Res*(ptrTable[idx_parent]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[idx_parent]->location.x/res + map_origin_x)));
        cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
    }

    cv::namedWindow("Mapping");
    cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);

}

void rrtTree::visualizeTree(std::vector<point> path){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map_original, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(x_init.y/res + map_origin_y)), (int)(Res*(x_init.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(x_goal.y/res + map_origin_y)), (int)(Res*(x_goal.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
        x1 = cv::Point((int)(Res*(ptrTable[i]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[i]->location.x/res + map_origin_x)));
        x2 = cv::Point((int)(Res*(ptrTable[idx_parent]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[idx_parent]->location.x/res + map_origin_x)));
        cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
    }

    thickness = 3;
    for(int i = 1; i < path.size(); i++) {
        x1 = cv::Point((int)(Res*(path[i-1].y/res + map_origin_y)), (int)(Res*(path[i-1].x/res + map_origin_x)));
        x2 = cv::Point((int)(Res*(path[i].y/res + map_origin_y)), (int)(Res*(path[i].x/res + map_origin_x)));
        cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
    }

    cv::namedWindow("Mapping");
    cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);

}


// TODO
// 1. Copy your implementation of member functions in Project Assignment #2
// 2. Implement generateRRTst
void rrtTree::addVertex(point x_new, point x_rand, int idx_near) {
    node* new_node = new node;
    new_node->idx = count;
    new_node->rand = x_rand;
    new_node->location = x_new;
    new_node->cost = ptrTable[idx_near]->cost + getC(x_new, ptrTable[idx_near]->location);
    new_node->idx_parent = idx_near;
    ptrTable[count++] = new_node;
}


point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
    point x_rand;
    x_rand.x = x_min + (x_max - x_min)*(0.125+(rand() % 80)/80.0);
    x_rand.y = y_min + (y_max - y_min)*(0.125+(rand() % 80)/80.0);
    return x_rand;
}


point rrtTree::newState(int idx_near, point x_rand, double MaxStep) {
    point x_near = ptrTable[idx_near]->location;
    point x_new;
    double alpha = sqrt(pow((x_rand.x-x_near.x),2)+pow((x_rand.y-x_near.y),2));
    if(alpha > MaxStep) alpha = MaxStep / alpha;
    else alpha = 1.0;
    /*
    else if(alpha > MaxStep/2 || ((x_rand.x == x_goal.x) && (x_rand.y == x_goal.y))) alpha = 1.0;
    else alpha = MaxStep*0.5 / alpha;
    */
    x_new.x = x_near.x + (x_rand.x-x_near.x)*alpha;
    x_new.y = x_near.y + (x_rand.y-x_near.y)*alpha;
    return x_new;
}


int rrtTree::nearestNeighbor(point x_rand) {
    double min_len = DBL_MAX;
    int idx = count;
    for(int i=count-1; i>-1; i--){
        point tmp_x = ptrTable[i]->location;
        double len = (tmp_x.x - x_rand.x)*(tmp_x.x - x_rand.x) + (tmp_x.y - x_rand.y)*(tmp_x.y - x_rand.y);
        if(min_len > len){
            min_len = len;
            idx = i;
        }
    }
    return idx;
}


bool rrtTree::isCollision(point x1, point x2) {
    double slope = (x1.y - x2.y)/(x1.x - x2.x);
    int sign = (slope>0)? 1: -1;
    int tmp_min_x, tmp_max_x, fin_y;
    double ori_x, ori_y;
    if(x1.x<x2.x){
        tmp_min_x = (int)round(x1.x/res + map_origin_x);
        tmp_max_x = (int)round(x2.x/res + map_origin_x);
        ori_x = x1.x/res + map_origin_x;
        ori_y = x1.y/res + map_origin_y;
        fin_y = (int)round(x2.y/res + map_origin_y);
    } else {
        tmp_min_x = (int)round(x2.x/res + map_origin_x);
        tmp_max_x = (int)round(x1.x/res + map_origin_x);
        ori_x = x2.x/res + map_origin_x;
        ori_y = x2.y/res + map_origin_y;
        fin_y = (int)round(x1.y/res + map_origin_y);
    }
    // Check if their is an obstacle on the path when i=tmp_min_x
    int cover_y_at_min_x = (int)round((tmp_min_x + 0.5 - ori_x)*slope + ori_y);
    for(int j=(int)round(ori_y); ; j+=sign){
        int tmp_state = map.at<uchar>(tmp_min_x, j);
        if(tmp_state < 255) return false;
        if(j == cover_y_at_min_x) break;
    }
    // Check if their is an obstacle on the path
    for(int cover_x=tmp_min_x+1; cover_x<tmp_max_x; ++cover_x){
        int cover_min_y = (int)round((cover_x - 0.5 - ori_x)*slope + ori_y);
        int cover_max_y = (int)round((cover_x + 0.5 - ori_x)*slope + ori_y);
        for(int cover_y=cover_min_y; ; cover_y+=sign){
            int tmp_state = map.at<uchar>(cover_x, cover_y);
            if(tmp_state < 255) return false;
            if(cover_y == cover_max_y) break;
        }
    }
    // Check if their is an obstacle on the path when i=tmp_max_x
    int cover_y_at_max_x = (int)round((tmp_max_x - 0.5 - ori_x)*slope + ori_y);
    for(int j=cover_y_at_max_x; ; j+=sign){
        int tmp_state = map.at<uchar>(tmp_max_x, j);
        if(tmp_state < 255) return false;
        if(j == fin_y) break;
    }
    return true;
}


std::vector<point> rrtTree::backtracking(){
    std::vector<point> path;
    int cnt = nearestNeighbor(x_goal);
    while(cnt != 0){
//        printf("\t\tBacktracking: node %d\n", cnt);
        path.insert(path.begin(), ptrTable[cnt]->location);
//        path.push_back(ptrTable[cnt]->location);
        cnt = ptrTable[cnt]->idx_parent;
    }
    path.push_back(ptrTable[0]->location);
//    visualizeTree(path);
    return path;
}


int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    srand(time(NULL));
    int cnt = 0;
    while(1){
        /*
        point x1, x2;
        x1.x=+1.1; x1.y=-7.2; x2.x=+1.2; x2.y=-7.1;
        if(isCollision(x1, x2)) printf("ok.\n");
        */
        point x_rand = (rand() % 5 == 0)? x_goal: randomState(x_max, x_min, y_max, y_min);
        int idx_near = nearestNeighbor(x_rand);
        point x_new = newState(idx_near, x_rand, MaxStep);
        if(!isCollision(x_new, ptrTable[idx_near]->location)) continue;
        addVertex(x_new, x_rand, idx_near);
        // Break if the closest vertex is close enough to x_goal
        if(++cnt == K){
                int nearest = nearestNeighbor(x_goal);
                point x_nearest = ptrTable[nearest]->location;
                /*
                double dist = sqrt(pow((x_goal.x-x_nearest.x), 2) + pow((x_goal.y-x_nearest.y), 2));
                printf("\t\tdist = %f\n", dist);
                if(dist < 0.04) return 0;
                */
                if((x_goal.x - x_nearest.x == 0) && (x_goal.y - x_nearest.y == 0)) return 0;
                else break;
        }
    }
    while(1){
        point x_rand = (rand() % 5 == 0)? x_goal: randomState(x_max, x_min, y_max, y_min);
        int idx_near = nearestNeighbor(x_rand);
        point x_new = newState(idx_near, x_rand, MaxStep);
        if(!isCollision(x_new, ptrTable[idx_near]->location)) continue;
        addVertex(x_new, x_rand, idx_near);
        // Break if the latest vertex is close enough to x_goal
        point x_nearest = ptrTable[count-1]->location;
        if((x_goal.x - x_nearest.x == 0) && (x_goal.y - x_nearest.y == 0)) return 0;
        if(++cnt == 20000) break;
    }
    return -1;
}


int rrtTree::generateRRTst(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep){
    // TODO
	// card(V) = size of V, the set of vertices
	// d = dimension of point
	double gamma = 1.0;	// Chosen by heuristic
    srand(time(NULL));
    int cnt = 0;
    while(1){
        point x_rand = (rand() % 5 == 0)? x_goal: randomState(x_max, x_min, y_max, y_min);
        int idx_nearest = nearestNeighbor(x_rand);
        point x_new = newState(idx_nearest, x_rand, MaxStep);	// mu = MaxStep
        if(!isCollision(x_new, ptrTable[idx_nearest]->location)) continue;
        //
        std::vector<int> X_nears = nearNeighbors(x_new, std::min(MaxStep, gamma*pow((log(count)/count),1/2)));
        std::vector<bool> X_nears_col;			// Check collision of (X_near_i, x_new)
        std::vector<double> X_nears_cost;		// Remember c(X_near_i, x_new)
        for(int i=0; i<X_nears.size(); ++i)
        	X_nears_col.push_back(isCollision(ptrTable[X_nears[i]]->location, x_new));
        for(int i=0; i<X_nears.size(); ++i)
        	X_nears_cost.push_back(getC(ptrTable[X_nears[i]]->location, x_new));
        // Connect along a minimun-cost path
        int idx_min = idx_nearest;
        double c_min = ptrTable[idx_nearest]->cost + getC(ptrTable[idx_nearest]->location, x_new);
        for(int i=0; i<X_nears.size(); ++i){
        	double tmp_c_min = ptrTable[X_nears[i]]->cost + X_nears_cost[i];
        	if(X_nears_col[i] && tmp_c_min < c_min){
        		idx_min = X_nears[i];
        		c_min = tmp_c_min;
        	}
         }
        addVertex(x_new, x_rand, idx_min);
        // Rewire the tree
        for(int i=0; i<X_nears.size(); ++i){
        	int idx_parent = ptrTable[X_nears[i]]->idx_parent;
        	if(X_nears_col[i] && (ptrTable[count]->cost + X_nears_cost[i] < ptrTable[X_nears[i]]->cost))
        		changeEdge(X_nears[i], count);
         }
        //
        if(++cnt > K-1){
        	int idx_near_goal = (cnt==K)? nearestNeighbor(x_goal): count;
        	point x_nearest = ptrTable[idx_near_goal]->location;
        	if((x_goal.x-x_nearest.x == 0) && (x_goal.y-x_nearest.y == 0)) return 0;
        	if(cnt == 20000) break;
        }
    }
    return -1;
}


double rrtTree::getC(point x1, point x2){
	// Default cost of a point: distance between a point and its parent
	// In general, Cost(p) = Cost(parent) + c(p, parent)
	// This function returns c(x1,x2)
	return sqrt((x1.x-x2.x)*(x1.x-x2.x) + (x1.y-x2.y)*(x1.y-x2.y));
}


void rrtTree::changeEdge(int idx, int idx_parent){
	ptrTable[idx]->idx_parent = idx_parent;
	ptrTable[idx]->cost = ptrTable[idx_parent]->cost + getC(ptrTable[idx]->location, ptrTable[idx_parent]->location);
}


std::vector<int> rrtTree::nearNeighbors(point x_new, double radius){
	std::vector<int> result;
    for(int i=count-1; i>-1; i--){
        point tmp_x = ptrTable[i]->location;
        double lensq = (tmp_x.x - x_new.x)*(tmp_x.x - x_new.x) + (tmp_x.y - x_new.y)*(tmp_x.y - x_new.y);
        if(radius*radius > lensq)
        	result.push_back(i);
    }
    /* L2 norm 대신 x_new+(+-radius, +-radius) 범위의 모든 point를 리턴
    for(int i=count-1; i>-1; i--){
        point tmp_x = ptrTable[i]->location;
        if(radius > max(tmp_x.x, x_new.x) - min(tmp_x.x, x_new.x)))
        	result.push_back(ptrTable[i]->location);
    }
    for(int j=result.size(); j>-1; j--){
        if(!(radius > max(result[j].y, x_new.y) - min(result[j].y, x_new.y))))
        	result.erase(result.begin()+j);
    }
    */
	return result;
}
