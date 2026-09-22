// Offline recorded-image calibration and saved-2D retriangulation. Never writes live configuration.
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include "lift/apriltag_marker.hpp"
#include "lift/calib_io.hpp"
#include "lift/floor_extrinsic_solver.hpp"
#include "lift/keypoint_format.hpp"
#include "lift/triangulator.hpp"

using namespace fitra;
namespace {
void require(bool ok, const std::string& message) { if (!ok) throw std::runtime_error(message); }
std::uint64_t fingerprint(std::istream& in) {
    std::uint64_t value = 14695981039346656037ULL;
    for (char c; in.get(c);) { value ^= static_cast<unsigned char>(c); value *= 1099511628211ULL; }
    require(in.eof(), "input read failed"); in.clear(); in.seekg(0); return value;
}
YAML::Node line(std::istream& in) {
    std::string text; require(static_cast<bool>(std::getline(in, text)), "truncated input");
    return YAML::Load(text);
}
lift::CalibrationSet calibration(const YAML::Node& cameras, const char* path) {
    require(cameras.IsSequence() && cameras.size()==3, "three cameras required");
    std::vector<std::string> ids;
    for (std::size_t c=0;c<3;++c) {
        require(cameras[c]["index"].as<int>()==static_cast<int>(c), "camera order mismatch");
        ids.push_back("cam"+std::to_string(cameras[c]["config_slot"].as<unsigned>()));
    }
    require(std::set<std::string>(ids.begin(),ids.end()).size()==3, "duplicate camera ID");
    auto result=lift::select_calib_cameras(lift::load_calibration(path),ids);
    lift::Triangulator(result).require_camera_ids(ids);
    for (std::size_t c=0;c<3;++c) require(
        result.cameras[c].intrinsics.width==cameras[c]["capture_width"].as<int>() &&
        result.cameras[c].intrinsics.height==cameras[c]["capture_height"].as<int>(), "calibration resolution mismatch");
    return result;
}
cv::Mat decode(const YAML::Node& frame, const YAML::Node& camera) {
    require(camera["raw_encoding"].as<std::string>()=="jpeg", "JPEG corpus required");
    auto path=frame["payload_path"].as<std::string>();
    auto offset=frame["offset"].as<std::uint64_t>(), size=frame["size"].as<std::uint64_t>();
    auto length=std::filesystem::file_size(path);
    require(size>0 && size<=64*1024*1024 && offset<=length && size<=length-offset, "invalid payload range");
    std::ifstream in(path,std::ios::binary); in.seekg(static_cast<std::streamoff>(offset));
    std::vector<unsigned char> bytes(size); in.read(reinterpret_cast<char*>(bytes.data()),size);
    require(in.good(),"payload read failed"); auto image=cv::imdecode(bytes,cv::IMREAD_COLOR);
    require(!image.empty() && image.cols==camera["capture_width"].as<int>() &&
            image.rows==camera["capture_height"].as<int>(),"JPEG dimensions mismatch"); return image;
}
void write_body(std::ostream& out, const lift::TriangulatedSkeleton& tri) {
    out << ",\"valid_joints\":" << tri.valid_joints << ",\"median_reproj_px\":";
    if(tri.valid_joints) out<<tri.median_reproj_px; else out<<"null";
    out<<",\"joints\":[";
    for(std::size_t k=0;k<26;++k) {
        if(k) out<<',';
        const auto& j=tri.skeleton.joints[k];
        if(!j.valid) { out<<"null"; continue; }
        require(std::isfinite(j.x)&&std::isfinite(j.y)&&std::isfinite(j.z)&&std::isfinite(j.score),"nonfinite 3D");
        out<<'['<<j.x<<','<<j.y<<','<<j.z<<','<<j.score<<']';
    }
    out<<"],\"joint_quality\":[";
    for(std::size_t k=0;k<26;++k) {
        if(k) out<<',';
        if(!tri.skeleton.joints[k].valid || k==0) { out<<"null"; continue; }
        require(std::isfinite(tri.reproj_error_px[k])&&std::isfinite(tri.max_ray_angle_deg[k]),"nonfinite geometry quality");
        out<<"{\"views\":"<<tri.view_count[k]<<",\"reproj_px\":"<<tri.reproj_error_px[k]
           <<",\"ray_angle_deg\":"<<tri.max_ray_angle_deg[k]<<'}';
    }
    out<<']';
}
void retriangulate(const char* plan_path, const char* raw_path, const char* calib_path, const char* output_path) {
    require(!std::filesystem::exists(output_path),"output exists");
    std::ifstream plan(plan_path), raw(raw_path); require(plan.good()&&raw.good(),"cannot open inputs");
    auto pb=fingerprint(plan), rb=fingerprint(raw); auto ph=line(plan), rh=line(raw);
    require(ph["schema"].as<std::string>()=="fitra_corpus_inference_plan_v1", "wrong plan schema");
    require(rh["schema"].as<std::string>()=="fitra_corpus_raw3d_v1" && !rh["postprocessing"].as<bool>() &&
            rh["kp_format"].as<std::string>()=="halpe26" && rh["plan_fnv1a64"].as<std::string>()==std::to_string(pb),
            "raw stage or plan binding mismatch");
    require(std::abs(rh["kp_conf_thresh"].as<double>()-.3)<1e-6 && rh["max_reproj_px"].as<double>()==6,
            "unsupported original triangulation settings");
    lift::set_active_keypoint_format(lift::KeypointFormat::Halpe26);
    lift::Triangulator tri(calibration(ph["cameras"],calib_path));
    std::ofstream out(output_path); require(out.good(),"cannot create output"); out<<std::setprecision(9);
    out<<"{\"schema\":\"fitra_corpus_retriangulation_v1\",\"plan_fnv1a64\":\""<<pb
       <<"\",\"source_fnv1a64\":\""<<rb<<"\"}\n";
    std::string text; std::int64_t prev_seq=-1,prev_time=-1; std::size_t count=0;
    while(std::getline(plan,text)) {
        auto g=YAML::Load(text), r=line(raw);
        auto seq=r["seq"].as<std::int64_t>(), t=r["t_ns"].as<std::int64_t>();
        require(seq>prev_seq && t>prev_time && seq==g["seq"].as<std::int64_t>() &&
                t==g["capture_max_ns"].as<std::int64_t>(),"lineage mismatch");
        bool ambiguous=r["ambiguous_detection"].as<bool>(), multi=false;
        require(r["observations_2d"].IsSequence() && r["observations_2d"].size()==3 &&
                r["detections"].IsSequence() && r["detections"].size()==3,"invalid observation array");
        std::array<infer::Person,3> people{}; std::vector<lift::PerCameraObservation> obs;
        for(int c=0;c<3;++c) {
            auto n=r["detections"][c].as<unsigned>(); multi |= n>1;
            auto points=r["observations_2d"][c];
            require(points.IsNull()==(ambiguous || n==0),"observation/detection mismatch");
            if(points.IsNull()) continue;
            require(points.IsSequence() && points.size()==26,"Halpe26 required");
            for(int k=0;k<26;++k) {
                require(points[k].IsSequence()&&points[k].size()==3,"invalid 2D keypoint");
                float x=points[k][0].as<float>(),y=points[k][1].as<float>(),s=points[k][2].as<float>();
                require(std::isfinite(x)&&std::isfinite(y)&&std::isfinite(s)&&s>=0,"invalid 2D value");
                people[c].kpts[k]={x,y,s};
            }
            obs.push_back({c,&people[c]});
        }
        require(ambiguous==multi,"ambiguous detection policy mismatch");
        auto body=ambiguous ? lift::TriangulatedSkeleton{} : tri.triangulate(obs);
        out<<"{\"seq\":"<<seq<<",\"t_ns\":"<<t; write_body(out,body); out<<"}\n";
        require(out.good(),"output write failed"); prev_seq=seq;prev_time=t;++count;
    }
    require(plan.eof()&&count>0,"empty/incomplete plan"); require(!std::getline(raw,text),"extra raw rows");
    require(raw.eof(),"raw read failed"); out.close(); require(out.good(),"output close failed");
}

struct Sample { std::int64_t seq; std::vector<lift::TagDetection> tags; };
struct Errors { double rms=0, maximum_frame_rms=0; int points=0, frames=0; std::set<int> ids; };
Errors errors(const std::vector<Sample>& samples, const lift::CameraCalibration& camera, const lift::FloorTagMap& map) {
    Errors result; double total=0; cv::Mat rvec;
    cv::Rodrigues(camera.extrinsics.T_cw(cv::Rect(0,0,3,3)),rvec);
    // fisheye::projectPoints requires contiguous vectors; a T_cw column ROI is strided.
    cv::Mat t=camera.extrinsics.T_cw(cv::Rect(3,0,1,3)).clone();
    for(const auto& sample:samples) {
        double frame_error=0;int frame_points=0;
        for(const auto& tag:sample.tags) {
            auto wc=map.world_corners(*map.find(tag.face_id)); std::vector<cv::Point3d> world;
            for(auto& p:wc) world.emplace_back(p.v[0],p.v[1],p.v[2]);
            std::vector<cv::Point2d> pixels;
            if(camera.intrinsics.is_fisheye()) cv::fisheye::projectPoints(world,pixels,rvec,t,camera.intrinsics.K,camera.intrinsics.dist);
            else cv::projectPoints(world,rvec,t,camera.intrinsics.K,camera.intrinsics.dist,pixels);
            for(int k=0;k<4;++k) { auto d=pixels[k]-cv::Point2d(tag.corners[k]); frame_error+=d.dot(d); }
            frame_points+=4; result.ids.insert(tag.face_id);
        }
        if(frame_points) { total+=frame_error;result.points+=frame_points;++result.frames;
            result.maximum_frame_rms=std::max(result.maximum_frame_rms,std::sqrt(frame_error/frame_points)); }
    }
    result.rms=result.points ? std::sqrt(total/result.points) : 0; return result;
}
void record_errors(cv::FileStorage& fs,const std::string& name,const Errors& e) {
    fs<<name<<"{"<<"points"<<e.points<<"frames"<<e.frames;
    if(e.points) fs<<"rms_px"<<e.rms<<"max_frame_rms_px"<<e.maximum_frame_rms;
    fs<<"tag_ids"<<"[";
    for(auto id:e.ids) fs<<id;
    fs<<"]"<<"}";
}
void calibrate(const char* plan_path,const char* calib_path,const char* map_path,const char* output_path) {
    namespace fsys=std::filesystem;
    require(!fsys::exists(output_path),"output directory exists");
    std::ifstream in(plan_path); require(in.good(),"cannot open plan"); auto binding=fingerprint(in); auto header=line(in);
    require(header["schema"].as<std::string>()=="fitra_corpus_calibration_plan_v1","wrong calibration plan");
    auto cameras=header["cameras"]; auto old=calibration(cameras,calib_path), candidate=old;
    auto map=lift::floor_tag_map_load(map_path); lift::MarkerBoardConfig board; board.use_clahe=true;
    for(auto& t:map.tags) board.faces.push_back({t.id,t.size_m});
    lift::AprilTagDetector detector(board);
    std::array<std::vector<Sample>,3> training,validation;
    fsys::create_directories(output_path);
    std::ofstream detections(fsys::path(output_path)/"detections.jsonl"); detections<<std::setprecision(9);
    std::int64_t previous_seq=-1, previous_time=-1; std::string text;
    while(std::getline(in,text)) {
        auto row=YAML::Load(text); auto seq=row["seq"].as<std::int64_t>(),t=row["capture_max_ns"].as<std::int64_t>();
        require(seq>previous_seq&&t>previous_time,"repeated/nonmonotonic calibration sample");
        auto split=row["split"].as<std::string>(); require(split=="train"||split=="validation","unknown split");
        require(row["frames"].IsSequence()&&row["frames"].size()==3,"incomplete sample");
        for(int c=0;c<3;++c) {
            const auto& intrinsic=old.cameras[c].intrinsics;
            require(row["frames"][c]["camera"].as<int>()==c,"camera mismatch");
            auto tags=detector.detect(decode(row["frames"][c],cameras[c]),intrinsic.K,intrinsic.dist,intrinsic.is_fisheye());
            tags.erase(std::remove_if(tags.begin(),tags.end(),[](const auto& tag){return !tag.pose_ok;}),tags.end());
            std::set<int> ids; for(const auto& tag:tags) require(ids.insert(tag.face_id).second,"duplicate tag in frame");
            (split=="train" ? training[c] : validation[c]).push_back({seq,tags});
            detections<<"{\"seq\":"<<seq<<",\"camera\":"<<c<<",\"split\":\""<<split<<"\",\"tags\":[";
            for(std::size_t i=0;i<tags.size();++i) {
                if(i) detections<<',';
                detections<<"{\"id\":"<<tags[i].face_id<<",\"corners\":[";
                for(int k=0;k<4;++k) { if(k) detections<<',';auto p=tags[i].corners[k];detections<<'['<<p.x<<','<<p.y<<']'; }
                detections<<"]}";
            }
            detections<<"]}\n";
        }
        previous_seq=seq;previous_time=t;
    }
    require(in.eof(),"plan read failed"); detections.close();require(detections.good(),"detection output failed");
    cv::FileStorage report((fsys::path(output_path)/"report.json").string(),cv::FileStorage::WRITE|cv::FileStorage::FORMAT_JSON);
    require(report.isOpened(),"report open failed");
    report<<"schema"<<"fitra_corpus_calibration_report_v1"<<"plan_fnv1a64"<<std::to_string(binding)
          <<"runtime_applied"<<false<<"cameras"<<"[";
    int updated=0;
    for(int c=0;c<3;++c) {
        std::map<int,std::vector<std::array<cv::Point2f,4>>> by_tag;
        for(const auto& sample:training[c]) for(const auto& tag:sample.tags) by_tag[tag.face_id].push_back(tag.corners);
        lift::FloorCameraInput ci; ci.cam_index=c; ci.K=old.cameras[c].intrinsics.K;
        ci.dist=old.cameras[c].intrinsics.dist;ci.fisheye=old.cameras[c].intrinsics.is_fisheye();
        for(const auto& entry:by_tag) if(entry.second.size()>=3) {
            lift::FloorTagObservation ob; ob.id=entry.first;
            for(int k=0;k<4;++k) { cv::Point2d sum{};for(auto& corners:entry.second) sum+=cv::Point2d(corners[k]);
                ob.corners[k]=sum*(1.0/entry.second.size()); }
            ci.obs.push_back(ob);
        }
        lift::FloorSolverOptions opts;opts.min_tags=2; auto solution=lift::solve_floor_extrinsics({ci},map,opts);
        auto prior=errors(validation[c],old.cameras[c],map);
        report<<"{"<<"camera"<<c<<"train_tags"<<static_cast<int>(ci.obs.size())<<"solver_ok"<<solution.ok;
        record_errors(report,"validation_original",prior);
        std::string reason="insufficient_or_failed_fit"; bool accepted=false;
        if(solution.ok) {
            auto proposal=old.cameras[c]; auto pose=solution.cameras[0].T_cam_world;
            cv::Mat T=cv::Mat::eye(4,4,CV_64F), R(pose.rot()),t(pose.trans());
            R.copyTo(T(cv::Rect(0,0,3,3)));t.copyTo(T(cv::Rect(3,0,1,3)));proposal.extrinsics.T_cw=T;
            cv::Mat center=-R.t()*t; proposal.extrinsics.camera_center_w={center.at<double>(0),center.at<double>(1),center.at<double>(2)};
            proposal.extrinsics.method="offline_corpus_floor_candidate";
            auto held=errors(validation[c],proposal,map);
            cv::Mat relative=R*old.cameras[c].extrinsics.T_cw(cv::Rect(0,0,3,3)).t(),rvec;cv::Rodrigues(relative,rvec);
            double angle=cv::norm(rvec)*180/CV_PI;
            double move=cv::norm(proposal.extrinsics.camera_center_w-old.cameras[c].extrinsics.camera_center_w);
            accepted=held.frames>=3 && held.ids.size()>=2 && held.rms<=3 && held.rms<prior.rms &&
                     held.maximum_frame_rms<=4 && move<=.1 && angle<=5;
            reason=accepted ? "heldout_improved_offline_candidate_only" : "heldout_or_small_change_gate_failed";
            report<<"fit_rms_px"<<solution.cameras[0].reproj_rms_px<<"planar_degenerate"<<solution.cameras[0].planar_degenerate
                  <<"center_delta_m"<<move<<"rotation_delta_deg"<<angle;
            record_errors(report,"validation_candidate",held);
            if(accepted) { candidate.cameras[c]=proposal;++updated; }
        }
        report<<"updated"<<accepted<<"reason"<<reason<<"}";
    }
    report<<"]"<<"updated_cameras"<<updated;report.release();
    // Explicit new filename only: never use write_calibration_versioned or touch latest pointers.
    lift::write_calibration((fsys::path(output_path)/"candidate.yaml").string(),candidate);
}
}
int main(int argc,char** argv) {
    try {
        if(argc==2 && std::string(argv[1])=="--help") {
            std::cout<<"corpus_geometry calibrate PLAN CALIB FLOOR_MAP NEW_DIRECTORY\n"
                       "corpus_geometry triangulate PLAN RAW_2D CALIB NEW_JSONL\n";return 0;
        }
        require(argc==6,"expected mode and four paths");
        if(std::string(argv[1])=="calibrate") calibrate(argv[2],argv[3],argv[4],argv[5]);
        else if(std::string(argv[1])=="triangulate") retriangulate(argv[2],argv[3],argv[4],argv[5]);
        else throw std::runtime_error("unknown mode");
        return 0;
    } catch(const std::exception& e) { std::cerr<<"corpus_geometry: "<<e.what()<<'\n';return 1; }
}
