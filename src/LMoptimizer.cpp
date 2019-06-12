//
// Created by echo on 18-7-25.
//
// 功能基本完成
//
#include "LMoptimizer.h"

using namespace std;
using namespace g2o;

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);

int main(int argc, char** argv)
{
    // Command line parsing
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");
    LMoptimizer lmoptimizer;
    lmoptimizer.setup(node,privateNode);
    lmoptimizer.spin();
    return 0;
}

void LMoptimizer::print4x4Matrix (const Eigen::Isometry3d & matrix)
{
    printf ("\e[0;32m""Rotation matrix of current TF :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void LMoptimizer::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode) {
    _loopclosure = node.subscribe<std_msgs::Bool>("/g2o_finish_once",1, &LMoptimizer::lmStart,this);
    _mappping = node.advertise<std_msgs::Bool>("/start_mapping",1);
}

void LMoptimizer::lmStart(const std_msgs::Bool::ConstPtr &loopClosure) {
    _g2ofinish  = loopClosure->data;
}

int LMoptimizer::spin() {
    ros::Rate rate(10);
    //bool status = ros::ok();
    while (ros::ok()) {
        ros::spinOnce();
        if(_g2ofinish){
            process();
        }
        rate.sleep();
    }
    if(!ros::ok())
    {///////////

    }
    return 0;
}

int LMoptimizer::process() {

    BlockSolverX::LinearSolverType * linearSolver = new LinearSolverCSparse<BlockSolverX::PoseMatrixType>();
    BlockSolverX* blockSolver = new BlockSolverX(linearSolver);
    OptimizationAlgorithmLevenberg* optimizationAlgorithm = new OptimizationAlgorithmLevenberg(blockSolver);

    // NOTE: We skip to fix a variable here, either this is stored in the file
    // itself or Levenberg will handle it.

    // create the optimizer to load the data and carry out the optimization
    SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    optimizer.setAlgorithm(optimizationAlgorithm);

    ifstream ifs(inputFilename.c_str());
    if (! ifs) {
        cerr << "unable to open " << inputFilename << endl;
        return 1;
    }
    optimizer.load(ifs);
    //todo Read every vertex
    vector<double> tmp;
//将来改一下
    for(int i = 0; i < optimizer.vertices().size(); i++){
        optimizer.vertex(i)->getEstimateData(tmp);
        Eigen::Matrix4d temp1 = Eigen::Matrix4d::Identity();
        Eigen::Isometry3d temp2 = Eigen::Isometry3d::Identity();
        Eigen::Quaterniond temp3 = Eigen::Quaterniond::Identity();
        temp3.x() = tmp[3];
        temp3.y() = tmp[4];
        temp3.z() = tmp[5];
        temp3.w() = tmp[6];

        temp2.pretranslate(Eigen::Vector3d (tmp[0],tmp[1],tmp[2]));
        temp2.rotate(temp3);
        //print4x4Matrix(temp2);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(maxIterations);

    if (outputFilename.size() > 0) {
        if (outputFilename == "-") {
            cerr << "saving to stdout";
            optimizer.save(cout);
        } else {
            cerr << "saving " << outputFilename << " ... ";
            optimizer.save(outputFilename.c_str());
        }
        cerr << "done." << endl;
    }
    _g2ofinish = false;
    _to_mapping.data = true;
    _mappping.publish(_to_mapping);
    return 0;
}
