//input 当前的里程计 闭环的两个时间戳 ndt给出的两个之间的变换矩阵
//output 找到哪两个点闭环
//print
//save g2o 的文件
//TODO information matrix 的确定
#include "g2oSaver.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "g2oSaver");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
  G2oSaver g2osaver;
  g2osaver.setup(node, privateNode);
  if(argc == 2){
    g2osaver.readg2o(argv[1]);
  }
  g2osaver.spin();
  cout << "" << endl;
}

void G2oSaver::addEstimate() {
  // concatenate all the odometry constraints to compute the initial state
  //链接所有的odom约束去计算初始状态
  for (size_t i = 0; i < _odometryEdges.size(); ++i) {
    EdgeSE3 *e = _edges[i]; //ok
    VertexSE3 *from = static_cast<VertexSE3 *>(e->vertex(0));
    VertexSE3 *to = static_cast<VertexSE3 *>(e->vertex(1));
    HyperGraph::VertexSet aux;
    aux.insert(from);
    e->initialEstimate(aux, to);
  }
}
void G2oSaver::print4x4Matrix(const Eigen::Isometry3d &matrix) {
  printf("\e[0;32m""Rotation matrix of current TF :\n");
  printf("    | %6.3f %6.3f %6.3f | \n",
         matrix(0, 0),
         matrix(0, 1),
         matrix(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n",
         matrix(1, 0),
         matrix(1, 1),
         matrix(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n",
         matrix(2, 0),
         matrix(2, 1),
         matrix(2, 2));
  printf("Translation vector :\n");
  printf("t = < %6.3f, %6.3f, %6.3f >\n\n",
         matrix(0, 3),
         matrix(1, 3),
         matrix(2, 3));
}

G2oSaver::G2oSaver() = default;

void G2oSaver::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode) {
  _odomLidar = node.subscribe<nav_msgs::Odometry>("/after_rotate",
                                                  1000,
                                                  &G2oSaver::lidarOdomHandler,
                                                  this);
  _NDT_temp = node.subscribe<nav_msgs::Odometry>("/ndt_loop_tf",
                                                 1,
                                                 &G2oSaver::ndtTrans,
                                                 this);
  _toLM = node.advertise<std_msgs::Bool>("/g2o_finish_once", 1);
  _loop_current = node.advertise<std_msgs::Header>("/loop_info_current", 1);
  _loop_precedent =
      node.advertise<std_msgs::Header>("/loop_info_precedent", 1);

  _information_loopclosure(0, 0) = 10;
  _information_loopclosure(1, 1) = 10;
  _information_loopclosure(2, 2) = 10;
  _information_loopclosure(3, 3) = 10;
  _information_loopclosure(4, 4) = 10;
  _information_loopclosure(5, 5) = 10;

  _information(0, 0) = 0.1;
  _information(1, 1) = 0.1;
  _information(2, 2) = 0.01;
  _information(3, 3) = 0.01;
  _information(4, 4) = 0.01;
  _information(5, 5) = 0.1;
  //串口的
//    fd = OpenDev(dev);
//    if (fd>0){
//        set_speed(fd, 115200);
//    }
}

int G2oSaver::spin() {
  ros::Rate rate(100);
  //bool status = ros::ok();
  while (ros::ok()) {
    ros::spinOnce();
//    process();
    //serialProcess();
    rate.sleep();
  }

  if (!ros::ok()) {///////////
    cout << "saving file" << endl;
    //addEstimate();
    //saveg2o(_odom_buffer);
    if (savefile()) {
      std::cout << "\033[31m" << "save g2o success" << "\033[37m" << std::endl;
      return 0;
    } else {
      std::cout << "\033[31m" << "g2o not saved" << "\033[37m" << std::endl;
      return 0;
    }
  }
  return 0;
}

void G2oSaver::process() {
  if (_newodom) {
    //TODO 绝对位置的计算
    _curpose = Eigen::Isometry3d::Identity();
    _cur_rotation_vector = Eigen::Quaterniond(_odomtmp.pose.pose.orientation.w,
                                              _odomtmp.pose.pose.orientation.x,
                                              _odomtmp.pose.pose.orientation.y,
                                              _odomtmp.pose.pose.orientation.z).matrix();
    _cur_trans_vector = Eigen::Vector3d(_odomtmp.pose.pose.position.x,
                                        _odomtmp.pose.pose.position.y,
                                        _odomtmp.pose.pose.position.z);
    _curpose.rotate(_cur_rotation_vector);
    _curpose.pretranslate(_cur_trans_vector);
    //Eigen::Isometry3d _curpose1 = Eigen::Isometry3d::Identity();
    saveg2o(_curpose);
    _odom_buffer.push_back(_curpose);
    pubPotentialLoopclosure();
  }
  if (_newodom) {
    _odomlast = _odomtmp;
  }
  _newodom = false;

  // cout << "Process OK" << endl;
}

void G2oSaver::readg2o(std::string g2o_path) {
  std::ifstream fin(g2o_path);
  if(!fin){
    return;
  }

  while(!fin.eof()){
    std::string name;
    fin>> name;
    if(name == "VERTEX_SE3:QUAT"){
      g2o::VertexSE3* v = new g2o::VertexSE3();
      int index = 0;
      fin>>index;

      v->setId( index );
      v->read(fin);
      _vertices.push_back(v);
    } else if(name == "EDGE_SE3:QUAT"){
      g2o::EdgeSE3* e = new g2o::EdgeSE3();
      int idx1, idx2;
      fin>>idx1>>idx2;
      e->setVertex( 0, _vertices.at(idx1) );
      e->setVertex( 1, _vertices.at(idx2) );
      e->read(fin);
      _edges.push_back(e);
    }
    if(!fin.good()) break;

  }
}

bool G2oSaver::saveg2o(Eigen::Isometry3d curr) {

  if (!_newloop) {
    //test here 17:31
    //cout<< "Transform matrix = \n"<<curr.matrix()<<endl;

    VertexSE3 *v = new VertexSE3;

    //std::shared_ptr<VertexSE3> v(new VertexSE3());
    //cout<<"  _vertexnum : before add :  "<<_vertexnum<<endl;
    v->setId(_vertexnum++);

    //cout << "dsasda" << endl << _vertexnum << endl;
    //添加顶点
    Eigen::Isometry3d t_v;
    //当前的t
    t_v = curr;
    v->setEstimate(t_v);
    if (_vertexnum == 1) {
      v->setFixed(true);
    }

    _vertices.push_back(v);

    // 生成边
    if (_vertexnum > 1) {
      cout << "  connect between " << _vertexnum - 2 << "---" << _vertexnum - 1
           << endl;

      // cout << "OSDAIO" << _vertices.size() << endl;

      VertexSE3 *prev = _vertices[_vertexnum - 2]; // debug
      VertexSE3 *cur = _vertices[_vertexnum - 1];

      cout << "  __vertices.size() : after add :  " << _vertices.size()
           << endl; //debug

      Eigen::Isometry3d t_e = prev->estimate().inverse() * cur->estimate();

      Eigen::Isometry3d ttmmpp = t_e; // debug
      EdgeSE3 *e = new EdgeSE3;

      e->setVertex(0, prev); //debug
      e->setVertex(1, cur); //debug
      e->setMeasurement(ttmmpp); //debug
      e->setInformation(_information);

      //正常边两个都有的
      _odometryEdges.push_back(e);
      _edges.push_back(e);
      // delete prev;
    }
  }
  //测试闭环

  /*if(_vertexnum == 105)
  {
      VertexSE3 *from = _vertices[0];  //debug
      VertexSE3 *to = _vertices[104];  //debug

      //_vertices[104]->setFixed(1);
      std::cout<<"fixed"<<std::endl;
      //Eigen::Isometry3d t = from->estimate().inverse() * to->estimate();
      Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      Eigen::Matrix4d a;

      a<<0.999313 ,-0.000049 ,0.037070 ,-5.842558,
      -0.000592 ,0.999850 ,0.017282 ,-4.376858,
      -0.037065 ,-0.017293 ,0.999163 ,0.245644,
      0.000000 ,0.000000 ,0.000000 ,1.000000;
      t = a.inverse();
      EdgeSE3 *e = new EdgeSE3;
      e->setVertex(0, from);
      e->setVertex(1, to);
      //e->1(t.inverse()); //debug
      e->setMeasurement(t);
      e->setInformation(_information_loopclosure);
      std::cout<<_information_loopclosure<<std::endl;
      //回环的只有一个
      _edges.push_back(e);
  }*/
  //测试闭环结束

  return true;
}

bool G2oSaver::savefile() {
  //ROS_ERROR("BEFORE SAVING G2O");
  if (_vertices.size() > 0) {
    //ROS_ERROR("SAVING G2O");
    ofstream fileOutputStream;
    //cerr << "Writing into " << _outFilename << endl;
    fileOutputStream.open(_outFilename.c_str());

    string vertexTag = Factory::instance()->tag(_vertices[0]);
    string edgeTag = Factory::instance()->tag(_edges[0]);
    //todo 去除下划线的操作? _outFilename = "-" ->cout
    ostream &fout = _outFilename != "-" ? fileOutputStream : cout;

    for (size_t i = 0; i < _vertices.size(); ++i) {
      VertexSE3 *v = _vertices[i];
      fout << vertexTag << " " << v->id() << " ";
      //fout << vertexTag << " " << _vertices[i]->id() << " ";
      v->write(fout);
      //_vertices[i]->write(fout);
      fout << endl;
    }

    for (size_t i = 0; i < _edges.size(); ++i) {
      fout << edgeTag << " "
           << static_cast<VertexSE3 *>(_edges[i]->vertex(0))->id()
           << " " << static_cast<VertexSE3 *>(_edges[i]->vertex(1))->id()
           << " ";
      _edges[i]->write(fout);
      fout << endl;
    }
    return true;
  } else {
    return false;
  }
}

void G2oSaver::lidarOdomHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry) {
  _newodom = true;
  //这里用来补偿kitti的旋转
  _odomtmp = *laserOdometry;
  odometryToEigen(_odomtmp, _odom_rotate_temp);
  _rotated = _odomtmp;
  eigenToOdom(_odom_rotate_temp, _rotated);
  //print4x4Matrix(_odom_rotate_temp);
  //_tfChanger.publish(rotated);
  _curpose = Eigen::Isometry3d::Identity();
  _cur_rotation_vector = Eigen::Quaterniond(_odomtmp.pose.pose.orientation.w,
                                            _odomtmp.pose.pose.orientation.x,
                                            _odomtmp.pose.pose.orientation.y,
                                            _odomtmp.pose.pose.orientation.z).matrix();
  _cur_trans_vector = Eigen::Vector3d(_odomtmp.pose.pose.position.x,
                                      _odomtmp.pose.pose.position.y,
                                      _odomtmp.pose.pose.position.z);
  _curpose.rotate(_cur_rotation_vector);
  _curpose.pretranslate(_cur_trans_vector);
  //Eigen::Isometry3d _curpose1 = Eigen::Isometry3d::Identity();
  saveg2o(_curpose);
  _odom_buffer.push_back(_curpose);
  pubPotentialLoopclosure();
}

void G2oSaver::poseEigenToTF(const Eigen::Isometry3d &e, tf::Pose &t) {
  tf::transformEigenToTF(e, t);
}

void G2oSaver::poseTFToEigen(const tf::Pose &t, Eigen::Isometry3d &e) {
  tf::transformTFToEigen(t, e);
}

void G2oSaver::transformEigenToTF(const Eigen::Isometry3d &e,
                                  tf::Transform &t) {
  tf::transformEigenToTF(e, t);
}

void G2oSaver::transformTFToEigen(const tf::Transform &t,
                                  Eigen::Isometry3d &e) {
  tf::transformTFToEigen(t, e);
}

void G2oSaver::odometryToEigen(nav_msgs::Odometry &o, Eigen::Isometry3d &e) {
  Eigen::Isometry3d is_test;
  is_test = Eigen::Isometry3d::Identity();
  tf::Transform tf_test;
  tf::poseMsgToTF(o.pose.pose, tf_test);
  tf::transformTFToEigen(tf_test, e);
}

void G2oSaver::eigenToOdom(Eigen::Isometry3d &e, nav_msgs::Odometry &o) {
  tf::Transform tf_test;
  tf_test.setIdentity();
  tf::transformEigenToTF(e, tf_test);
  tf::poseTFToMsg(tf_test, o.pose.pose);
}

void G2oSaver::ndtTrans(const nav_msgs::Odometry::ConstPtr &loopClosure) {
  //////todo 不对 计算闭环的真正tf
  if (loopClosure->twist.covariance.elems[0] == 1
      && loopClosure->twist.covariance.elems[2] == 1) {
    //todo 2018/8/16 run here

    //存下tf
    Eigen::Isometry3d _odom_loop = Eigen::Isometry3d::Identity();
    nav_msgs::Odometry transedOdomFunction;
    transedOdomFunction = *loopClosure;
    odometryToEigen(transedOdomFunction, _odom_loop);
    _trans_loopclosure.push_back(_odom_loop);
    //得到匹配贞的编号
    _vertxfromnum.push_back((int) loopClosure->twist.covariance.elems[1]);
    _vertextonum.push_back((int) loopClosure->twist.covariance.elems[3]);
    saveg2o();
  }
}

bool G2oSaver::saveg2o() {
  //保存闭环

  cout << "****** from:   " << _vertxfromnum.back() << "   to:   "
       << _vertextonum.back() << endl;
  cout << "****** Loop closure (NDT) trasnform is :" << endl;
  cout << _trans_loopclosure.back().matrix() << endl;

  if (_vertices.size() > _vertxfromnum.back()
      && _vertices.size() > _vertextonum.back()) {
    VertexSE3 *from = _vertices.at(_vertxfromnum.back());
    VertexSE3 *to = _vertices.at(_vertextonum.back());
    EdgeSE3 *e = new EdgeSE3();
    e->setVertex(0, from);
    e->setVertex(1, to);
    //todo calculateRealTF 用公式计算真正的tf 根据odometry和ndt

    e->setMeasurement(calculateRealTF(_trans_loopclosure.back(), *from, *to));
    e->setInformation(_information_loopclosure);
    _edges.push_back(e);
    savefile();
    std_msgs::Bool tolm;
    tolm.data = 1;
    _toLM.publish(tolm);
  } else {
    ROS_ERROR(
        "Current loop closure frame does not belong to the vertex, add loop failed");
  }
  //cerr<<"_vertices.size():  "<<_vertices.size()<<"_vertxfromnum.back():  "<<_vertxfromnum.back()
  //    <<" _vertextonum.back()  "<< _vertextonum.back()<<endl;

  return false;
}

void G2oSaver::pubPotentialLoopclosure() {
  if (_odom_buffer.size() % _k_search_interval == 0) {
    int pre_potential = -1;
    // find nearest frame
    int cur_potential = _odom_buffer.size() + _k_search_from;
    double distance = _k_search_radius;
    for (int i = 0; i < int(_odom_buffer.size()) - _k_search_before; i++) {
      Eigen::Vector3d cur_xy = _odom_buffer.at(cur_potential).translation();
      Eigen::Vector3d pre_xy = _odom_buffer.at(i).translation();
      cur_xy.z() = pre_xy.z() = 0;
      Eigen::Vector3d diff = cur_xy-pre_xy;
          - _odom_buffer.at(i).translation();
      if (diff.norm() < distance) {
        distance = diff.norm();
        pre_potential = i;
      }
    }
    if (pre_potential == -1) {
      return;
    }
    std_msgs::Header msgs;
    msgs.seq = cur_potential;
    std::cout << "current potential: " << msgs.seq << std::endl;
    _loop_current.publish(msgs);
    msgs.seq = pre_potential;
    std::cout << "precedent potential: " << msgs.seq << std::endl;
    _loop_precedent.publish(msgs);
  }
}

void G2oSaver::set_speed(int fd, int speed) {
  int i;
  int status;
  struct termios Opt;

  tcgetattr(fd, &Opt);
  for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
    if (speed == name_arr[i]) {
      tcflush(fd, TCIOFLUSH);
      cfsetispeed(&Opt, speed_arr[i]);
      cfsetospeed(&Opt, speed_arr[i]);
      status = tcsetattr(fd, TCSANOW, &Opt);
      if (status != 0)
        perror("tcsetattr fd1");
      return;
    }
    tcflush(fd, TCIOFLUSH);

  }
}

int G2oSaver::OpenDev(char *Dev) {
  int fd = open(Dev, O_RDWR);
  if (-1 == fd) {
    perror("Can't Open Serial Port");
    return -1;
  } else
    return fd;
}

int G2oSaver::serialProcess() {

  if ((nread = read(fd, buff, sizeof(buff))) > 0) {
    string a;
    printf("recv_message: %s \n", buff);

    sprintf(buff_char, "%s", buff);
    a = buff_char;
    cout << a << endl;
    memset(buff, 0, sizeof(buff));
    memset(buff_char, 0, sizeof(buff_char));

  }
  return 0;
}
//
Eigen::Isometry3d G2oSaver::calculateRealTF(Eigen::Isometry3d Tndt,
                                            VertexSE3 from,
                                            VertexSE3 to) {
  auto from_tmp = from.estimate();
  auto to_tmp = to.estimate();
  Eigen::Isometry3d t_e = Eigen::Isometry3d::Identity();
  t_e = Tndt * from_tmp;
  t_e = t_e.inverse() * to_tmp;
//    t_e = from_tmp.inverse() * to_tmp;
//
//    t_e = t_e*Tndt;
  cout << "real tf :" << endl;
  print4x4Matrix(t_e);
  return t_e;
}



