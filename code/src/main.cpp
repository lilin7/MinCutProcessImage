/**COMP 6651 P1
 * Lin Li
 * Student ID: 40044486
 */

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
using namespace cv;
using namespace std;

//forward declaration
struct Vertex;

/*Struct Edge, create Edge object*/
struct Edge {
	//constructor
	Edge(Vertex& b, Vertex& e, int w) : begin(b), end(e), weight(w), residual(w) {}

	Vertex& begin; //begin vertex of this edge
	Vertex& end; //end vertex of this edge
	int weight; //weight of this edge
	int residual; //residual weight of this edge
};

/*Struct Vertex, create Vertex object*/
struct Vertex {
	//constructor
	Vertex() : visited(false), parent(nullptr) {
		outgoingEdgesOfVertex.reserve(4); 
	}
	//method to set if this vertex is visited
	void setVisited(bool v = true) {
		visited = v;
	}

	Edge& getTo(Vertex& v) {
		for (Edge& e : outgoingEdgesOfVertex) {
			if (&e.end == &v) return e;
		}
	}

	vector<Edge> outgoingEdgesOfVertex;  //edges from this vertex
	bool visited; //if this vertex is visited, for BFS/fordFulkerson
	Vertex* parent; //parent vertex of this vertex, for BFS/fordFulkerson
};

/*Struct Graph, create Graph object*/
struct Graph {
	//constructor
	Graph(int w, int h) : allVertices(w*h), width(w), height(h){
	}

	// clear the visited status of a graph
	void clearVisited() {
		superSource.visited = false;
		superSink.visited = false;
		for (Vertex& v : allVertices) {
			v.visited = false;
		}
	}

	//a method to return a vertex at certain location in a given picture
	Vertex& getAt(int i, int j) {
		return allVertices.at(i + j*width);
	}
	// actual data storage
	vector<Vertex> allVertices;
	Vertex superSource;
	Vertex superSink;

	// graph info
	int width, height;
	
	vector<Vertex*> sourceVertices; //info of source Vertices given by config file
	vector<Vertex*> sinkVertices; // info of sink Vertices given by config file
	vector<Edge*> allEdges; //info of all edges
};

/**
* BFS algorithm to process a graph, start from virtual super source, end at virtual super sink
* return true if there is a path from source to sink
*/
bool bfs(Graph& graph, vector<Vertex*>& path) {
	graph.clearVisited(); //each time before using BFS, clear the graph's status 
	path.clear(); //each time before using BFS, clear the path's status 
	queue<Vertex*> q; //a queue of vertices
	q.push(&graph.superSource); //put supersource into queue
	graph.superSource.setVisited(); //set supersource as visited
 
	while (!q.empty()) { //process queue
		Vertex& cur = *q.front();
		q.pop();

		for( Edge& e : cur.outgoingEdgesOfVertex ) {
			if (e.end.visited == false && e.residual > 0) {
				q.push(&e.end);
				e.end.parent = &cur;
				e.end.visited = true;
			}
		}
	}

	for (Vertex* cur = &graph.superSink; cur; cur = cur->parent) {
		path.insert(path.begin(), cur);
	}
	return (graph.superSink.visited == true); //return true if there is a path from source to sink
}

/**
* FordFulkerson method to process a graph, 
*/
void fordFulkerson(Graph& graph) {
	vector<Vertex*> path;
	//use BFS method to find a path and adjust weight in residual graph
	while (bfs(graph, path)) { //loop when there is a path from superSink to superSource
		int path_flow = INT_MAX;
		auto superSink = path.end() - 1;
		for (auto it = path.begin(); it != superSink; ++it) {
			path_flow = min(path_flow, (*it)->getTo(*(*(it + 1))).residual);
		}
		//calculate residual weight of all edges of this path
		for (auto it = path.begin(); it != superSink; ++it){
			(*it)->getTo(*(*(it + 1))).residual -= path_flow;
			(*(it + 1))->getTo(*(*it)).residual += path_flow;
		}
	}
}

/**
* A method to connect two vertices (pixels in a given image) with 2 edges of opposite direction, 
* assign weight (count by the difference of RGB values of the two pixels) to both edges
*/
void connectPixel(Vertex& l, Vec3b& lp, Vertex& r, Vec3b& rp) {
	int norm = abs(lp[0] - rp[0]) + abs(lp[1] - rp[1]) + abs(lp[2] - rp[2]);
	int w = int( 2048 * exp( (-1*norm*norm)/8 ));
	l.outgoingEdgesOfVertex.push_back(Edge(l, r, w));
	r.outgoingEdgesOfVertex.push_back(Edge(r, l, w));
}

int main(int argc, char** argv) {
	if (argc != 4) {
		cout << "Usage: ../seg input_image initialization_file output_mask" << endl;
		return -1;
	}
	// Load the input image
	// the image should be a 3 channel image by default but we will double check that in teh seam_carving
	Mat in_image;
	in_image = imread(argv[1]/*, CV_LOAD_IMAGE_COLOR*/);
	if (!in_image.data) {
		cout << "Could not load input image!!!" << endl;
		return -1;
	}
	if (in_image.channels() != 3) {
		cout << "Image does not have 3 channels!!! " << in_image.depth() << endl;
		return -1;
	}
	// the output image
	Mat out_image = in_image.clone();
	ifstream configFile(argv[2]);
	if (!configFile) {
		cout << "Could not load initial mask file!!!" << endl;
		return -1;
	}

	int width = in_image.cols;
	int height = in_image.rows;

	//use constructor to create a graph with input image file
	Graph graph(width, height);

	int n;
	configFile >> n;
	//read in config file, use the data to create graph
	for (int i = 0; i < n; ++i) {
		int x, y, t;
		configFile >> x >> y >> t;
		if (x < 0 || x >= width || y < 0 || y >= height) {
			cout << "Invalid pixel mask!" << endl;
			return -1;
		}
		
		Vertex& v = graph.getAt(x, y); // read the coordinate given in the config file 

		if (t == 1) { //set those pixels as source in the graph, connect to virtual superSource
			v.outgoingEdgesOfVertex.push_back(Edge(v, graph.superSource, 0));
			//set the flow from virtual superSource to this source as infinite (actually use max of int)
			graph.superSource.outgoingEdgesOfVertex.push_back(Edge(graph.superSource, v, INT_MAX));
		}
		else if ( t== 0 ) { //set those pixels as sink in the graph, connect to virtual superSink
			//set the flow from this source to virtual superSource as infinite (actually use max of int)
			v.outgoingEdgesOfVertex.push_back(Edge(v, graph.superSink, INT_MAX));
			graph.superSink.outgoingEdgesOfVertex.push_back(Edge(graph.superSink, v, 0));
		}
		else {
			assert(false); // t must be 0 or 1
		}
	}

	// connect each pixel to its 4 neighbours (if the pixel is at the edge of the image, could be 2 or 3 neighbours)
	for (int h = 0; h < height; ++h) {
		for (int w = 0; w < width; ++w) {
			Vertex& v = graph.getAt(w, h);
			Vec3b& vp = in_image.at<Vec3b>(h, w);
			if (w > 0) {
				Vertex& l = graph.getAt(w-1, h);
				connectPixel(v, vp, l, in_image.at<Vec3b>(h, w - 1));
			}
			if (w < width-1) {
				Vertex& r = graph.getAt(w + 1, h);
				connectPixel(v, vp, r, in_image.at<Vec3b>(h, w + 1));
			}
			if (h > 0) {
				Vertex& b = graph.getAt(w, h -1);
				connectPixel(v, vp, b, in_image.at<Vec3b>(h-1, w));
			}
			if (h < height-1) {
				Vertex& t = graph.getAt(w, h+1);
				connectPixel(v, vp, t, in_image.at<Vec3b>(h+1,w));
			}
		}
	}

	//call method fordFulkerson to process this graph and do min-cut
	fordFulkerson(graph);

	Vec3b pixel;
	//loop every pixel of the graph, find the ones can be visited from source, set as foreground;
	//find the ones can't be visited from source (which belongs to the sink category), set as background
	for (int h = 0; h < height; ++h) {
		for (int w = 0; w < width; ++w) {
			Vertex& v = graph.getAt(w, h);
			pixel[0] = 0;
			pixel[1] = 0;
			pixel[2] = 0;

			if (v.visited == 1) {
				pixel[0] = 255;
				pixel[1] = 255;
				pixel[2] = 255;
			}
			else {
				pixel[0] = 0;
				pixel[1] = 0;
				pixel[2] = 0;
			}
			out_image.at<Vec3b>(h, w) = pixel;
		}
	}

	// write it on disk
	imwrite(argv[3], out_image);

	// also display them both
	namedWindow("Original image", WINDOW_AUTOSIZE);
	namedWindow("Show Marked Pixels", WINDOW_AUTOSIZE);
	imshow("Original image", in_image);
	imshow("Show Marked Pixels", out_image);
	waitKey(0);
	return 0;
}
