#include <string>
#include <geometry_msgs/Pose.h>
#include <unordered_map>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
using namespace std;

class ObjectInfo {
private:
    std::string objPath;
    unsigned int id;
    unsigned int kid;
    std::string frame_id;
    std::string name;
    geometry_msgs::Pose pose;
    double scale_x;
    double scale_y;
    double scale_z;
    double r;
    double g;
    double b;
    bool use_embedded_materials;
public:
    inline void setObjPath(std::string p) {objPath=p;}
    inline std::string getObjPath() {return objPath;}

    inline unsigned int getArucoID() const {return id;}
    inline unsigned int getKauthamID() const {return kid;}
    inline std::string get_frame_id() const {return frame_id;}
    inline std::string getname() const {return name;}
    inline double getx() const {return pose.position.x;}
    inline double gety() const {return pose.position.y;}
    inline double getz() const {return pose.position.z;}
    inline double getqx() const {return pose.orientation.x;}
    inline double getqy() const {return pose.orientation.y;}
    inline double getqz() const {return pose.orientation.z;}
    inline double getqw() const {return pose.orientation.w;}
    inline bool get_use_embeded_materials() const {return use_embedded_materials;}
    inline double getr() const {return r;}
    inline double getg() const {return g;}
    inline double getb() const {return b;}
    inline double getscale_x() const {return scale_x;}
    inline double getscale_y() const {return scale_y;}
    inline double getscale_z() const {return scale_z;}
    inline geometry_msgs::Pose getPose() const {return pose;}

    inline void setArucoID(unsigned int i) {id=i;}
    inline void setKauthamID(unsigned int i) {kid=i;}
    inline void set_frame_id(std::string s) {frame_id=s;}
    inline void setname(std::string s) {name=s;}
    inline void setx(double v) {pose.position.x=v;}
    inline void sety(double v) {pose.position.y=v;}
    inline void setz(double v) {pose.position.z=v;}
    inline void setqx(double v) {pose.orientation.x=v;}
    inline void setqy(double v) {pose.orientation.y=v;}
    inline void setqz(double v) {pose.orientation.z=v;}
    inline void setqw(double v) {pose.orientation.w=v;}
    inline void setr(double v) {r=v;}
    inline void setg(double v) {g=v;}
    inline void setb(double v) {b=v;}
    inline void setscale_x(double v) {scale_x=v;}
    inline void setscale_y(double v) {scale_y=v;}
    inline void setscale_z(double v) {scale_z=v;}
    inline void set_use_embedded_materials(bool t) {use_embedded_materials = t;}

    ObjectInfo() = default;
    ObjectInfo(const ObjectInfo &obj2);
};

ObjectInfo::ObjectInfo (const ObjectInfo &obj2) {
    setArucoID(obj2.getArucoID());
    setKauthamID(obj2.getKauthamID());
    set_frame_id(obj2.get_frame_id());
    setname(obj2.getname());
    setx(obj2.getx());
    sety(obj2.gety());
    setz(obj2.getz());
    setqx(obj2.getqx());
    setqy(obj2.getqy());
    setqz(obj2.getqz());
    setqw(obj2.getqw());
    setr(obj2.getr());
    setg(obj2.getg());
    setb(obj2.getb());
    setscale_x(obj2.getscale_x());
    setscale_y(obj2.getscale_y());
    setscale_z(obj2.getscale_z());
    set_use_embedded_materials(obj2.get_use_embeded_materials());
};

class Cell {
    private:
        double x, y;
    public:
        inline void setx(double v) { x = v ;}
        inline void sety(double v) { y = v ;}
        
        inline double getx() {return x; }
        inline double gety() {return y; }
        
        Cell() = default;
        Cell(double v , double z){
            x = v; y = z;
        };
};

class Castle{
    private:
        bool a = true;
        bool b = true;
    public:
        inline void seta(bool v) {a = v;}
        inline void setb(bool v) {b = v;}

        inline bool geta() {return a;}
        inline bool getb() {return b;}
};

Castle castleoperation;

class Checkmate{
    private:
        bool a = false; 
        bool b = false;
    public:
        inline void seta(bool v) {a = v;}
        inline void setb(bool v) {b = v;}

        inline bool geta() {return a;}
        inline bool getb() {return b;}
};

Checkmate cm;

std::vector<float> conf(14);
std::map<unsigned int, unsigned int> aruco2rviz_map; 
std::map<unsigned int, unsigned int> rviz2aruco_map; 
std::map<unsigned int, unsigned int> aruco2kautham_map; 
std::map<unsigned int, unsigned int> kautham2aruco_map; 
std::map<unsigned int, std::string> aruco2ff_map;
std::map<std::string,unsigned int> ff2aruco_map;
std::map<std::string, Cell> mymap;


int attached[2]  = {-1,-1}; 

tf2_ros::Buffer tfBuffer;

std::vector<ObjectInfo> objects;


void SetChessWorld()
{
    ObjectInfo obj;
    double chessboard_height = 0.04;
    double short_height = 0.04; //pawns
    double middle_height = 0.06; //tower, horses and knights
    double tall_height = 0.08; //queen, king  
    
    //Pieces will be located w.r.t /chess_frame located at the center of the board onthe top surface

    //object id set top 100.
    //aruco markers on the board corners (100 to 103)

    objects.clear();
    
    //chessboard dimension 0.46x0.46x0.01 (scale fixed such that it is obtained from a 0.05x0.05x0.05 cube)
    obj.setObjPath("package://chesslab_setup/models/chessboard/meshes/chessboard.dae");
    obj.setArucoID(100);
    obj.setKauthamID(0);
    obj.setname("CHESSBOARD");
    obj.set_frame_id("/chess_frame");
    obj.setx(0);
    obj.sety(0);
    obj.setz(-chessboard_height/2.0); //reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(9.2); //46.0/5.0
    obj.setscale_y(9.2);//46.0/5.0
    obj.setscale_z(0.8);//1.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae file
    objects.push_back(obj);


    //Id made coincident with aruco marker id
    //Black pawns: 201 to 208
    //Black towers: 209, 210
    //Black horses: 211, 212
    //Black knights: 213, 214
    //Black queen and kings: 215, 216

    //pawn dimension 0.03x0.03x0.04
    //pawnB1
    obj.setObjPath("package://chesslab_setup/models/pawnB1/meshes/pawnB1.dae");
    obj.setArucoID(201);
    obj.setKauthamID(1);
    obj.setname("BLACK_PAWN_1");
    obj.set_frame_id("/chess_frame");
    obj.setx(0.125);
    obj.sety(0.175);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(0.8);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //pawnB2
    obj.setObjPath("package://chesslab_setup/models/pawnB2/meshes/pawnB2.dae");
    obj.setArucoID(202);
    obj.setKauthamID(2);
    obj.setname("BLACK_PAWN_2");
    obj.set_frame_id("/chess_frame");
    obj.setx(0.125);
    obj.sety(0.125);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(0.8);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //pawnB3
    obj.setObjPath("package://chesslab_setup/models/pawnB3/meshes/pawnB3.dae");
    obj.setArucoID(203);
    obj.setKauthamID(3);
    obj.setname("BLACK_PAWN_3");
    obj.set_frame_id("/chess_frame");
    obj.setx(0.125);
    obj.sety(0.075);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(0.8);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //pawnB4
    obj.setObjPath("package://chesslab_setup/models/pawnB4/meshes/pawnB4.dae");
    obj.setArucoID(204);
    obj.setKauthamID(4);
    obj.setname("BLACK_PAWN_4");
    obj.set_frame_id("/chess_frame");
    obj.setx(0.125);
    obj.sety(0.025);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(0.8);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //pawnB5
    obj.setObjPath("package://chesslab_setup/models/pawnB5/meshes/pawnB5.dae");
    obj.setArucoID(205);
    obj.setKauthamID(5);
    obj.setname("BLACK_PAWN_5");
    obj.set_frame_id("/chess_frame");
    obj.setx(0.125);
    obj.sety(-0.025);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(0.8);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //pawnB6
    obj.setObjPath("package://chesslab_setup/models/pawnB6/meshes/pawnB6.dae");
    obj.setArucoID(206);
    obj.setKauthamID(6);
    obj.setname("BLACK_PAWN_6");
    obj.set_frame_id("/chess_frame");
    obj.setx(0.125);
    obj.sety(-0.075);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(0.8);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //pawnB7
    obj.setObjPath("package://chesslab_setup/models/pawnB7/meshes/pawnB7.dae");
    obj.setArucoID(207);
    obj.setKauthamID(7);
    obj.setname("BLACK_PAWN_7");
    obj.set_frame_id("/chess_frame");
    obj.setx(0.125);
    obj.sety(-0.125);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(0.8);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //pawnB8
    obj.setObjPath("package://chesslab_setup/models/pawnB8/meshes/pawnB8.dae");
    obj.setArucoID(208);
    obj.setKauthamID(8);
    obj.setname("BLACK_PAWN_8");
    obj.set_frame_id("/chess_frame");
    obj.setx(0.125);
    obj.sety(-0.175);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(0.8);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);

    //TowerB1
    obj.setObjPath("package://chesslab_setup/models/towerB1/meshes/towerB1.dae");
    obj.setArucoID(209);
    obj.setKauthamID(9);
    obj.setname("BLACK_TOWER_1");
    obj.set_frame_id("/chess_frame");
    obj.setx(0.175);
    obj.sety(-0.175);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(1.2);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //TowerB2
    obj.setObjPath("package://chesslab_setup/models/towerB2/meshes/towerB2.dae");
    obj.setArucoID(210);
    obj.setKauthamID(10);
    obj.setname("BLACK_TOWER_2");
    obj.set_frame_id("/chess_frame");
    obj.setx(0.175);
    obj.sety(0.175);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(1.2);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //HorseB1
    obj.setObjPath("package://chesslab_setup/models/horseB1/meshes/horseB1.dae");
    obj.setArucoID(211);
    obj.setKauthamID(11);
    obj.setname("BLACK_HORSE_1");
    obj.set_frame_id("/chess_frame");
    obj.setx(0.175);
    obj.sety(-0.125);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(1.2);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //HorseB2
    obj.setObjPath("package://chesslab_setup/models/horseB2/meshes/horseB2.dae");
    obj.setArucoID(212);
    obj.setKauthamID(12);
    obj.setname("BLACK_HORSE_2");
    obj.set_frame_id("/chess_frame");
    obj.setx(0.175);
    obj.sety(0.125);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(1.2);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //KnightB1
    obj.setObjPath("package://chesslab_setup/models/knightB1/meshes/knightB1.dae");
    obj.setArucoID(213);
    obj.setKauthamID(13);
    obj.setname("BLACK_KNIGHT_1");
    obj.set_frame_id("/chess_frame");
    obj.setx(0.175);
    obj.sety(-0.075);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(1.2);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //KnightB2
    obj.setObjPath("package://chesslab_setup/models/knightB2/meshes/knightB2.dae");
    obj.setArucoID(214);
    obj.setKauthamID(14);
    obj.setname("BLACK_KNIGHT_2");
    obj.set_frame_id("/chess_frame");
    obj.setx(0.175);
    obj.sety(0.075);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(1.2);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //QueenB
    obj.setObjPath("package://chesslab_setup/models/queenB/meshes/queenB.dae");
    obj.setArucoID(215);
    obj.setKauthamID(15);
    obj.setname("BLACK_QUEEN");
    obj.set_frame_id("/chess_frame");
    obj.setx(0.175);
    obj.sety(0.025);
    obj.setz(tall_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(1.6);//8.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //KingB
    obj.setObjPath("package://chesslab_setup/models/kingB/meshes/kingB.dae");
    obj.setArucoID(216);
    obj.setKauthamID(16);
    obj.setname("BLACK_KING");
    obj.set_frame_id("/chess_frame");
    obj.setx(0.175);
    obj.sety(-0.025);
    obj.setz(tall_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(-M_PI/4.0));
    obj.setqw(cos(-M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(1.6);//8.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);


    //White pawns: 301 to 308
    //White towers: 309, 310
    //White horses: 311, 312
    //White knights: 313, 314
    //White queen and kings: 315, 316

    //pawnW1
    obj.setObjPath("package://chesslab_setup/models/pawnW1/meshes/pawnW1.dae");
    obj.setArucoID(301);
    obj.setKauthamID(17);
    obj.setname("WHITE_PAWN_1");
    obj.set_frame_id("/chess_frame");
    obj.setx(-0.125);
    obj.sety(-0.175);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(0.8);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //pawnW2
    obj.setObjPath("package://chesslab_setup/models/pawnW2/meshes/pawnW2.dae");
    obj.setArucoID(302);
    obj.setKauthamID(18);
    obj.setname("WHITE_PAWN_2");
    obj.set_frame_id("/chess_frame");
    obj.setx(-0.125);
    obj.sety(-0.125);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(0.8);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //pawnW3
    obj.setObjPath("package://chesslab_setup/models/pawnW3/meshes/pawnW3.dae");
    obj.setArucoID(303);
    obj.setKauthamID(19);
    obj.setname("WHITE_PAWN_3");
    obj.set_frame_id("/chess_frame");
    obj.setx(-0.125);
    obj.sety(-0.075);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(0.8);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //pawnW4
    obj.setObjPath("package://chesslab_setup/models/pawnW4/meshes/pawnW4.dae");
    obj.setArucoID(304);
    obj.setKauthamID(20);
    obj.setname("WHITE_PAWN_4");
    obj.set_frame_id("/chess_frame");
    obj.setx(-0.125);
    obj.sety(-0.025);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(0.8);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //pawnW5
    obj.setObjPath("package://chesslab_setup/models/pawnW5/meshes/pawnW5.dae");
    obj.setArucoID(305);
    obj.setKauthamID(21);
    obj.setname("WHITE_PAWN_5");
    obj.set_frame_id("/chess_frame");
    obj.setx(-0.125);
    obj.sety(0.025);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(0.8);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //pawnW6
    obj.setObjPath("package://chesslab_setup/models/pawnW6/meshes/pawnW6.dae");
    obj.setArucoID(306);
    obj.setKauthamID(22);
    obj.setname("WHITE_PAWN_6");
    obj.set_frame_id("/chess_frame");
    obj.setx(-0.125);
    obj.sety(0.075);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(0.8);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //pawnW7
    obj.setObjPath("package://chesslab_setup/models/pawnW7/meshes/pawnW7.dae");
    obj.setArucoID(307);
    obj.setKauthamID(23);
    obj.setname("WHITE_PAWN_7");
    obj.set_frame_id("/chess_frame");
    obj.setx(-0.125);
    obj.sety(0.125);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(0.8);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //pawnW8
    obj.setObjPath("package://chesslab_setup/models/pawnW8/meshes/pawnW8.dae");
    obj.setArucoID(308);
    obj.setKauthamID(24);
    obj.setname("WHITE_PAWN_8");
    obj.set_frame_id("/chess_frame");
    obj.setx(-0.125);
    obj.sety(0.175);
    obj.setz(short_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(0.8);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);

    //TowerB1
    obj.setObjPath("package://chesslab_setup/models/towerW1/meshes/towerW1.dae");
    obj.setArucoID(309);
    obj.setKauthamID(25);
    obj.setname("WHITE_TOWER_1");
    obj.set_frame_id("/chess_frame");
    obj.setx(-0.175);
    obj.sety(-0.175);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(1.2);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //TowerB2
    obj.setObjPath("package://chesslab_setup/models/towerW2/meshes/towerW2.dae");
    obj.setArucoID(310);
    obj.setKauthamID(26);
    obj.setname("WHITE_TOWER_2");
    obj.set_frame_id("/chess_frame");
    obj.setx(-0.175);
    obj.sety(0.175);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(1.2);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //HorseB1
    obj.setObjPath("package://chesslab_setup/models/horseW1/meshes/horseW1.dae");
    obj.setArucoID(311);
    obj.setKauthamID(27);
    obj.setname("WHITE_HORSE_1");
    obj.set_frame_id("/chess_frame");
    obj.setx(-0.175);
    obj.sety(-0.125);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(1.2);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //HorseB2
    obj.setObjPath("package://chesslab_setup/models/horseW2/meshes/horseW2.dae");
    obj.setArucoID(312);
    obj.setKauthamID(28);
    obj.setname("WHITE_HORSE_2");
    obj.set_frame_id("/chess_frame");
    obj.setx(-0.175);
    obj.sety(0.125);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(1.2);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //KnightB1
    obj.setObjPath("package://chesslab_setup/models/knightW1/meshes/knightW1.dae");
    obj.setArucoID(313);
    obj.setKauthamID(29);
    obj.setname("WHITE_KNIGHT_1");
    obj.set_frame_id("/chess_frame");
    obj.setx(-0.175);
    obj.sety(-0.075);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(1.2);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //KnightB2
    obj.setObjPath("package://chesslab_setup/models/knightW2/meshes/knightW2.dae");
    obj.setArucoID(314);
    obj.setKauthamID(30);
    obj.setname("WHITE_KNIGHT_2");
    obj.set_frame_id("/chess_frame");
    obj.setx(-0.175);
    obj.sety(0.075);
    obj.setz(middle_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(1.2);//6.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //QueenB
    obj.setObjPath("package://chesslab_setup/models/queenW/meshes/queenW.dae");
    obj.setArucoID(315);
    obj.setKauthamID(31);
    obj.setname("WHITE_QUEEN");
    obj.set_frame_id("/chess_frame");
    obj.setx(-0.175);
    obj.sety(-0.025);
    obj.setz(tall_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(1.6);//8.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);
    //KingW
    obj.setObjPath("package://chesslab_setup/models/kingW/meshes/kingW.dae");
    obj.setArucoID(316);
    obj.setKauthamID(32);
    obj.setname("WHITE_KING");
    obj.set_frame_id("/chess_frame");
    obj.setx(-0.175);
    obj.sety(0.025);
    obj.setz(tall_height/2.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI/4.0));
    obj.setqw(cos(M_PI/4.0));
    obj.setscale_x(0.6);//3.0/5.0
    obj.setscale_y(0.6);//3.0/5.0
    obj.setscale_z(1.6);//8.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    objects.push_back(obj);

 
    //set the index maps
    //kautham indices are automtically set according to the order in which they are written in the kautham problem xml file
    //rviz indices are set in the order they are loaded in the objects vector
    //rviz indices have been made coincident to kautham indices, although they could differ
    //the following maps relate the aruco markers with the rviz and kautham indices
    for(int i=0; i<objects.size();i++){
        //mymap.insert ( std::pair<char,int>('a',100) );
        aruco2kautham_map.insert(std::pair<unsigned int,unsigned int>(objects[i].getArucoID(), objects[i].getKauthamID()));
        kautham2aruco_map.insert(std::pair<unsigned int,unsigned int>(objects[i].getKauthamID(), objects[i].getArucoID()));
        aruco2rviz_map.insert(std::pair<unsigned int,unsigned int>(objects[i].getArucoID(), i));
        rviz2aruco_map.insert(std::pair<unsigned int,unsigned int>(i, objects[i].getArucoID()));
        aruco2ff_map.insert(std::pair<unsigned int,std::string>(objects[i].getArucoID(), objects[i].getname()));
        ff2aruco_map.insert(std::pair<std::string,unsigned int>(objects[i].getname(),objects[i].getArucoID()));
    }
 
    mymap.insert(std::pair<std::string,Cell>("A1",Cell(-0.175, -0.175)));
    mymap.insert(std::pair<std::string,Cell>("A2",Cell(-0.125, -0.175)));
    mymap.insert(std::pair<std::string,Cell>("A3",Cell(-0.075, -0.175)));
    mymap.insert(std::pair<std::string,Cell>("A4",Cell(-0.025, -0.175)));
    mymap.insert(std::pair<std::string,Cell>("A5",Cell(0.025, -0.175)));
    mymap.insert(std::pair<std::string,Cell>("A6",Cell(0.075, -0.175)));
    mymap.insert(std::pair<std::string,Cell>("A7",Cell(0.125, -0.175)));
    mymap.insert(std::pair<std::string,Cell>("A8",Cell(0.175, -0.175)));
    mymap.insert(std::pair<std::string,Cell>("B1",Cell(-0.175, -0.125)));
    mymap.insert(std::pair<std::string,Cell>("B2",Cell(-0.125, -0.125)));
    mymap.insert(std::pair<std::string,Cell>("B3",Cell(-0.075, -0.125)));
    mymap.insert(std::pair<std::string,Cell>("B4",Cell(-0.025,- 0.125)));
    mymap.insert(std::pair<std::string,Cell>("B5",Cell(0.025, -0.125)));
    mymap.insert(std::pair<std::string,Cell>("B6",Cell(0.075, -0.125)));
    mymap.insert(std::pair<std::string,Cell>("B7",Cell(0.125, -0.125)));
    mymap.insert(std::pair<std::string,Cell>("B8",Cell(0.175, -0.125)));
    mymap.insert(std::pair<std::string,Cell>("C1",Cell(-0.175, -0.075)));
    mymap.insert(std::pair<std::string,Cell>("C2",Cell(-0.125, -0.075)));
    mymap.insert(std::pair<std::string,Cell>("C3",Cell(-0.075, -0.075)));
    mymap.insert(std::pair<std::string,Cell>("C4",Cell(-0.025, -0.075)));
    mymap.insert(std::pair<std::string,Cell>("C5",Cell(0.025, -0.075)));
    mymap.insert(std::pair<std::string,Cell>("C6",Cell(0.075, -0.075)));
    mymap.insert(std::pair<std::string,Cell>("C7",Cell(0.125, -0.075)));
    mymap.insert(std::pair<std::string,Cell>("C8",Cell(0.175, -0.075)));
    mymap.insert(std::pair<std::string,Cell>("D1",Cell(-0.175, -0.025)));
    mymap.insert(std::pair<std::string,Cell>("D2",Cell(-0.125, -0.025)));
    mymap.insert(std::pair<std::string,Cell>("D3",Cell(-0.075, -0.025)));
    mymap.insert(std::pair<std::string,Cell>("D4",Cell(-0.025, -0.025)));
    mymap.insert(std::pair<std::string,Cell>("D5",Cell(0.025, -0.025)));
    mymap.insert(std::pair<std::string,Cell>("D6",Cell(0.075, -0.025)));
    mymap.insert(std::pair<std::string,Cell>("D7",Cell(0.125, -0.025)));
    mymap.insert(std::pair<std::string,Cell>("D8",Cell(0.175, -0.025)));
    mymap.insert(std::pair<std::string,Cell>("E1",Cell(-0.175, 0.025)));
    mymap.insert(std::pair<std::string,Cell>("E2",Cell(-0.125, 0.025)));
    mymap.insert(std::pair<std::string,Cell>("E3",Cell(-0.075, 0.025)));
    mymap.insert(std::pair<std::string,Cell>("E4",Cell(-0.025, 0.025)));
    mymap.insert(std::pair<std::string,Cell>("E5",Cell(0.025, 0.025)));
    mymap.insert(std::pair<std::string,Cell>("E6",Cell(0.075, 0.025)));
    mymap.insert(std::pair<std::string,Cell>("E7",Cell(0.125, 0.025)));
    mymap.insert(std::pair<std::string,Cell>("E8",Cell(0.175, 0.025)));
    mymap.insert(std::pair<std::string,Cell>("F1",Cell(-0.175, 0.075)));
    mymap.insert(std::pair<std::string,Cell>("F2",Cell(-0.125, 0.075)));
    mymap.insert(std::pair<std::string,Cell>("F3",Cell(-0.075, 0.075)));
    mymap.insert(std::pair<std::string,Cell>("F4",Cell(-0.025, 0.075)));
    mymap.insert(std::pair<std::string,Cell>("F5",Cell(0.025, 0.075)));
    mymap.insert(std::pair<std::string,Cell>("F6",Cell(0.075, 0.075)));
    mymap.insert(std::pair<std::string,Cell>("F7",Cell(0.125, 0.075)));
    mymap.insert(std::pair<std::string,Cell>("F8",Cell(0.175, 0.075)));
    mymap.insert(std::pair<std::string,Cell>("G1",Cell(-0.175, 0.125)));
    mymap.insert(std::pair<std::string,Cell>("G2",Cell(-0.125, 0.125)));
    mymap.insert(std::pair<std::string,Cell>("G3",Cell(-0.075, 0.125)));
    mymap.insert(std::pair<std::string,Cell>("G4",Cell(-0.025, 0.125)));
    mymap.insert(std::pair<std::string,Cell>("G5",Cell(0.025, 0.125)));
    mymap.insert(std::pair<std::string,Cell>("G6",Cell(0.075, 0.125)));
    mymap.insert(std::pair<std::string,Cell>("G7",Cell(0.125, 0.125)));
    mymap.insert(std::pair<std::string,Cell>("G8",Cell(0.175, 0.125)));
    mymap.insert(std::pair<std::string,Cell>("H1",Cell(-0.175, 0.175)));
    mymap.insert(std::pair<std::string,Cell>("H2",Cell(-0.125, 0.175)));
    mymap.insert(std::pair<std::string,Cell>("H3",Cell(-0.075, 0.175)));
    mymap.insert(std::pair<std::string,Cell>("H4",Cell(-0.025, 0.175)));
    mymap.insert(std::pair<std::string,Cell>("H5",Cell(0.025, 0.175)));
    mymap.insert(std::pair<std::string,Cell>("H6",Cell(0.075, 0.175)));
    mymap.insert(std::pair<std::string,Cell>("H7",Cell(0.125, 0.175)));
    mymap.insert(std::pair<std::string,Cell>("H8",Cell(0.175, 0.175)));
}

