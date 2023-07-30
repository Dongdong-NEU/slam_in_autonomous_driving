//
// Created by xiang on 2021/11/5.
//

#include <glog/logging.h>
#include <iomanip>

#include "ch3/imu_integration.h"
#include "common/io_utils.h"
#include "tools/ui/pangolin_window.h"

DEFINE_string(imu_txt_path, "./data/ch3/10.txt", "数据文件路径");
DEFINE_bool(with_ui, true, "是否显示图形界面");

/// 本程序演示如何对IMU进行直接积分
/// 该程序需要输入data/ch3/下的文本文件，同时它将状态输出到data/ch3/state.txt中，在UI中也可以观察到车辆运动
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_imu_txt_path.empty()) {
        return -1;
    }

    sad::TxtIO io(FLAGS_imu_txt_path);

    // 该实验中，我们假设零偏已知
    Vec3d gravity(0, 0, -9.8);  // 重力方向
    Vec3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
    Vec3d init_ba(-0.165205, 0.0926887, 0.0058049);

    sad::IMUIntegration imu_integ(gravity, init_bg, init_ba);

    std::shared_ptr<sad::ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<sad::ui::PangolinWindow>();
        ui->Init();
    }

    /// 记录结果，将结果都存到fout中；
    // save_result 是一个函数对象，接收匿名函数，相当于给匿名函数取别名；
    auto save_result = [](std::ofstream& fout, double timestamp, const Sophus::SO3d& R, const Vec3d& v,
                          const Vec3d& p) {
        // save_vec3是一个函数对象，接收匿名函数，相当于给匿名函数取别名；
        auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { 
            fout << v[0] << " " << v[1] << " " << v[2] << " "; };
        // save_quat是一个函数对象，接受匿名函数，相当于给匿名函数取别名；
        auto save_quat = [](std::ofstream& fout, const Quatd& q) {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };

        // 先将时间戳写上；
        fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
        save_vec3(fout, p);
        save_quat(fout, R.unit_quaternion());
        save_vec3(fout, v);
        fout << std::endl;
    };

    std::ofstream fout("./data/ch3/state.txt");
    // 将匿名函数作为参数传入，该匿名函数被函数对象接受；
    // 匿名函数的捕获列表说明那些变量将要在匿名函数中将被用到；
    // SetIMUProcessFunc将返回对象本身，通常情况下，这种方式用于实现方法链式调用；
    io.SetIMUProcessFunc([&imu_integ, &save_result, &fout, &ui](const sad::IMU& imu) {
          // 实现imu的积分；
          imu_integ.AddIMU(imu);
          // 保存imu积分后的结果；
          save_result(fout, imu.timestamp_, imu_integ.GetR(), imu_integ.GetV(), imu_integ.GetP());
          if (ui) {
              ui->UpdateNavState(imu_integ.GetNavState());
              usleep(1e2);
          }
      }).Go(); // 这个.Go()函数用于读取接下来的IMU数据；

    // 打开了可视化的话，等待界面退出
    while (ui && !ui->ShouldQuit()) {
        usleep(1e4);
    }

    if (ui) {
        ui->Quit();
    }

    return 0;
}