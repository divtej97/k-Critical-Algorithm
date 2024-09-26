#include<bits/stdc++.h>
#include<iomanip>
using namespace std;

struct treeNode{
    int value;
    vector <treeNode*> children;
};

void compute_shortest_path(int** dist, int** next, int n){
    for(int k = 1; k <= n; k++){
        for(int i = 1; i <= n; i++){
            for(int j = 1; j <= n; j++){
                if(dist[i][k] != INT_MAX && dist[k][j] != INT_MAX && dist[i][j] > dist[i][k] + dist[k][j]){
                    dist[i][j] = dist[i][k] + dist[k][j];
                    next[i][j] = next[i][k];
                }
            }
        }
    }
}

vector<int> return_path(int si, int ei, int** next, int n){
    vector<int> p;
    int curr = si;
    while(curr != ei){
        p.push_back(curr);
        curr = next[curr][ei];
    }
    p.push_back(ei);
    return p;
}

void print_graph(int** g, int n){
    cout<<" ";
    for(int i = 1; i <= n; i++)
        cout<<setw(4)<<i;
    cout<<endl;
    for(int i = 1; i <= n; i++){
        cout<<i;
        for(int j = 1; j <= n; j++){
            if(g[i][j] == INT_MAX)  cout<<setw(4)<<"-1";
            else    cout<<setw(4)<<g[i][j];
        }
        cout<<endl;
    }
}

int return_critical_node(int** dist, int n, int* controller_assigned){
    int max_sum_dist = INT_MIN, critical_node = -1;
    for(int i = 1; i <= n; i++){
        if(controller_assigned[i] == -1){
            int dist_sum = 0;
            for(int j = 1; j <= n; j++){
                if(dist[i][j] != INT_MAX){
                    dist_sum += dist[i][j];
                }
            }
            if(dist_sum > max_sum_dist){
                max_sum_dist = dist_sum;
                critical_node = i;
            }
        }
    }
    return critical_node;
}

void print_controller_tree(treeNode* curr, vector<int> &path_yet){
    path_yet.push_back(curr->value);
    //if leaf node, print path from controller
    if(curr->children.size() == 0){
        for(int i = 0; i < path_yet.size(); i++){
            cout<<path_yet[i]<<"->";
        }
        cout<<endl;
    }
    else{
        for(int i = 0; i < curr->children.size(); i++){
            print_controller_tree(curr->children[i], path_yet);
        }
    }
    path_yet.pop_back();
}

void add_node_to_controller_tree(treeNode* curr, vector<int> &p, int curr_index){
    if(curr_index >= p.size()){
        return;
    }
    int i = 0;
    for(; i < curr->children.size(); i++){
        if(curr->children[i]->value == p[curr_index]){
            add_node_to_controller_tree(curr->children[i], p, curr_index + 1);
            return;
        }
    }
    if(i == curr->children.size()){
        treeNode* child = new treeNode;
        child->value = p[curr_index];
        curr->children.push_back(child);
        add_node_to_controller_tree(child, p, curr_index + 1);
    }
}

//this function creates tree with all nodes i.e. before selecting best candidate. So it does not take dreq into account
treeNode* create_controller_tree(int cont, int** dist, int** next, int n){
    treeNode* c = new treeNode;
    c->value = cont;
    //add all other nodes through their shortest path to the controller
    for(int i = 1; i <= n; i++){
        if(i != cont && dist[cont][i] != INT_MAX){
            vector<int> p = return_path(cont, i, next, n);
            add_node_to_controller_tree(c, p, 1);
        }
    }
    return c;
}

//this function creates the tree for the best controller. i.e. after selecting the best candidate. i.e it selects nodes which satisfy dreq
treeNode* create_best_controller_tree(int cont, int** dist, int**next, int n, int dreq){
    treeNode* c = new treeNode;
    c->value = cont;
    //adds all other nodes through their shortest path which satisfy the shortest path to be < dreq
    for(int i = 1; i <= n; i++){
        if(i != cont && dist[cont][i] <= dreq){
            vector<int> p = return_path(cont, i, next, n);
            add_node_to_controller_tree(c, p, 1);
        }
    }
    return c;
}

float candidate_height(treeNode* root){
    if(root->children.size() == 0){
        return 0;
    }
    float height = INT_MIN;
    for(int i = 0; i < root->children.size(); i++){
        height = max(height, candidate_height(root->children[i]));
    }
    return height + 1;
}

float return_hi(treeNode* root, int i){
    queue<treeNode*> q;
    q.push(root);
    q.push(NULL);
    int level = 0;
    float nodes = 0;
    while(level <= i && q.size() > 0){
        treeNode* curr = q.front();
        q.pop();
        if(curr == NULL){
            if(q.size() == 0)
                break;
            else{
                q.push(NULL);
                level++;
            }
        }
        else{
            if(level > 0)
                nodes++;
            for(int i = 0; i < curr->children.size(); i++){
                q.push(curr->children[i]);
            }
        }
    }
    return nodes;
}

float return_dmax(treeNode* curr, treeNode* root, int** dist){
    if(curr->children.size() == 0){
        return dist[root->value][curr->value];
    }
    float temp = INT_MIN;
    for(int i = 0; i < curr->children.size(); i++){
        temp = max(temp, return_dmax(curr->children[i], root, dist));
    }
    return temp;
}

float return_theta(treeNode* root, float best_height, float dreq, int n, int** dist){
    float curr_height = candidate_height(root);
    float gamma = curr_height / best_height;
    float dg = root->children.size();
    //taking i value as 2 i.e. finding nodes covered in 2 hops
    float hi = return_hi(root, 2);
    float node_connectivity = dg / (n - hi);
    float dmax = return_dmax(root, root, dist);
    float path_weight = dmax / dreq;
    return gamma * node_connectivity + (1 - gamma) * path_weight;
}

void mark_controller(treeNode* curr, treeNode* cont, int* controller_assigned, map <int, treeNode*> &m){
    controller_assigned[curr->value] = cont->value;
    m[curr->value] = cont;
    for(int i = 0; i < curr->children.size(); i++){
        mark_controller(curr->children[i], cont, controller_assigned, m);
    }
}

void remove_children_from_graph(treeNode* curr, int** dist, int n){
    for(int i = 1; i <= n; i++){
        if(i != curr->value){
            dist[curr->value][i] = INT_MAX;
        }
    }
    for(int i = 1; i <= n; i++){
        if(i != curr->value){
            dist[i][curr->value] = INT_MAX;
        }
    }
    for(int i = 0; i < curr->children.size(); i++){
        remove_children_from_graph(curr->children[i], dist, n);
    }
}

void assign_controller(int** dist, int** next, int n, int* controller_assigned, map <int, treeNode*> &m, vector<treeNode*> &all_controllers, int dreq){
    int cn = return_critical_node(dist, n, controller_assigned);
    cout<<endl<<"Critical node now - "<<cn<<endl;
    if(cn == -1){

    }
    //find possible controllers for critical node
    vector<int> poss_controllers;
    for(int i = 1; i <= n; i++){
        if(i != cn && dist[i][cn] <= dreq){
            poss_controllers.push_back(i);
        }
    }

    //create trees for all candidate controllers for evaluation
    vector<treeNode*> candidates;
    for(vector<int>::iterator it = poss_controllers.begin(); it != poss_controllers.end(); it++){
        treeNode* head = create_controller_tree(*it, dist, next, n);
        vector<int> temp;
//        cout<<endl;
//        print_controller_tree(head, temp);
//        cout<<endl;
        candidates.push_back(head);
    }

    //evaluate candidates and select best
    float best_height = INT_MIN;
    for(int i = 0; i < candidates.size(); i++)
        best_height = max(best_height, candidate_height(candidates[i]));
    float best_theta = INT_MIN;
    treeNode* best;
    for(int i = 0; i < candidates.size(); i++){
        float curr_theta = return_theta(candidates[i], best_height, dreq, n, dist);
        if(curr_theta > best_theta){
            best_theta = curr_theta;
            best = candidates[i];
        }
    }

    //set best controller to control nodes that satisfy dreq
    best = create_best_controller_tree(best->value, dist, next, n, dreq);
    vector<int> p;
    print_controller_tree(best, p);

    //set controller_assigned for nodes in this tree
    mark_controller(best, best, controller_assigned, m);

    //remove these nodes from graph i.e. make distance as MAX in dist
    remove_children_from_graph(best, dist, n);

    //add this controller in map of controllers
    all_controllers.push_back(best);
}

int main(){
    //no of nodes in graph
    int n, e;
    cout<<"Enter the number of nodes in network"<<endl;
    cin>>n;
    cout<<"Enter the number of links in network"<<endl;
    cin>>e;

    int** g = new int*[n + 1];
    int** dist = new int*[n + 1];
    int** next = new int*[n + 1];
    for(int i = 0; i <= n; i++){
        g[i] = new int[n + 1];
        dist[i] = new int[n + 1];
        next[i] = new int[n + 1];
        for(int j = 0; j <= n; j++){
            if(i == j){
                g[i][j] = 0;
                dist[i][j] = 0;
                next[i][j] = -1;
            }
            else{
                g[i][j] = INT_MAX;
                dist[i][j] = INT_MAX;
                next[i][j] = INT_MAX;
            }
        }
    }
    cout<<"Enter the nodes having a link between them and the propagation delay"<<endl;
    for(int i = 1; i <= e; i++){
        int f, s, delay;
        cin>>f>>s>>delay;
        g[f][s] = delay;
        g[s][f] = delay;
        dist[f][s] = delay;
        dist[s][f] = delay;
        next[f][s] = s;
        next[s][f] = f;
    }

    compute_shortest_path(dist, next, n);

    //copying dist to orig_dist - maintains shortest distance between all pairs of nodes
    int** orig_dist = new int*[n + 1];
    for(int i = 0; i <= n; i++){
        orig_dist[i] = new int[n + 1];
        for(int j = 0; j <= n; j++)
            orig_dist[i][j] = dist[i][j];
    }

    cout<<endl<<"Shortest path matrix is "<<endl;
    print_graph(dist, n);

    int dreq;
    cout<<endl<<"Enter propagation delay constraint for communication between a switch and its controller"<<endl;
    cin>>dreq;
    //controller array will check if every node is assigned a controller
    int controller_assigned[n + 1];
    controller_assigned[0] = 0;
    for(int i = 1; i <= n; i++)
        controller_assigned[i] = -1;

    //m maintains all pointer to controller for nodes
    map <int, treeNode*> m;

    //maintains all controllers
    vector<treeNode*> all_controllers;

    //assign controllers to nodes not in cluster yet
    for(int i = 1; i <= n; i++){
        if(controller_assigned[i] == -1){
            assign_controller(dist, next, n, controller_assigned, m, all_controllers, dreq);
            compute_shortest_path(dist, next, n);
        }
    }

    return 0;
}
