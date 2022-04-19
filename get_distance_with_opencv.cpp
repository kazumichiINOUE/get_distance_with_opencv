/*!
  ¥example get_distance.cpp 距離データを取得する
  ¥author Satofumi KAMIMURA

  さらにopencv で連続的に描画する
  Author Kazumichi INOUE <k.inoue@oyama-ct.ac.jp>
  Create 2022-04-16 20:54:32

  $Id$
  */

#include <iostream>
#include <atomic>
#include <opencv2/opencv.hpp>

#include <signal.h>
#include <unistd.h>

#include "Urg_driver.h"
#include "Connection_information.h"
#include "math_utilities.h"

using namespace qrk;

// 終了処理へ入るフラグ
std::atomic<bool> stop_flag(false);

// SIGINT捕獲時にフラグをセットする
void signal_handler(int sig, siginfo_t *info, void *ctx) {
  stop_flag.store(true);
  std::cout << "Get SIGINT\n";
}

int main(int argc, char *argv[]) {
  // SIGINTの捕獲設定
  struct sigaction sa_sigabrt;
  memset(&sa_sigabrt, 0, sizeof(sa_sigabrt));
  sa_sigabrt.sa_sigaction = signal_handler;
  sa_sigabrt.sa_flags = SA_SIGINFO;
  if (sigaction(SIGINT, &sa_sigabrt, NULL) < 0) {
    exit(1);
  }

  // LiDAR接続設定
  Connection_information information(argc, argv);  // 引数なしでUSB接続, -e <IP address>でイーサネット接続

  // LiDAR接続
  Urg_driver urg;
  if (!urg.open(information.device_or_ip_name(),
        information.baudrate_or_port_number(),
        information.connection_type())) {
    std::cout << "Urg_driver::open(): " << information.device_or_ip_name() << ": " << urg.what() << std::endl;
    return 1;
  }

  // データの取得範囲を変更する場合
  urg.set_scanning_parameter(urg.deg2step(-120), urg.deg2step(+120), 0);

  // opencvの準備
  cv::Size IMG_SIZE = cv::Size(800, 800);  // Width x Height
  std::cout << IMG_SIZE.width << " " << IMG_SIZE.height << "\n";
  cv::Scalar BACK_COLOR = cv::Scalar(182, 182, 182);
  cv::Scalar X_AXIS_COLOR = cv::Scalar(200, 0, 0);
  cv::Scalar Y_AXIS_COLOR = cv::Scalar(0, 200, 0);
  cv::Scalar OBSTACLE_COLOR = cv::Scalar(0, 0, 200);
  cv::Mat img_back = cv::Mat(IMG_SIZE, CV_8UC3, BACK_COLOR);
  cv::line(img_back,
      cv::Point(0, 6000/12000.0*IMG_SIZE.height), cv::Point(IMG_SIZE.width, 6000/12000.0*IMG_SIZE.height),
      X_AXIS_COLOR, 1, cv::LINE_AA);
  cv::line(img_back,
      cv::Point(1000/7000.0 * IMG_SIZE.width, 0), cv::Point(1000/7000.0 * IMG_SIZE.width, IMG_SIZE.height),
      Y_AXIS_COLOR, 1, cv::LINE_AA);

  // データ取得
  urg.start_measurement(Urg_driver::Distance, Urg_driver::Infinity_times, 0);
  for (;;) {
    // SIGINT を捕獲したらループを抜ける
    if (stop_flag.load() == true) {
      std::cout << "\tStop action\n";
      break;
    }

    std::vector<long> data;
    long time_stamp = 0;

    if (!urg.get_distance(data, &time_stamp)) {
      std::cout << "Urg_driver::get_distance(): " << urg.what() << std::endl;
      return 1;
    }

    // 描画用画像の準備
    cv::Mat img;
    img_back.copyTo(img);

    //print_data(urg, data, time_stamp);
    // 全てのデータの X-Y の位置を表示
    long min_distance = urg.min_distance();
    long max_distance = urg.max_distance();
    size_t data_n = data.size();
    for (size_t i = 0; i < data_n; ++i) {
      long l = data[i];
      if ((l <= min_distance) || (l >= max_distance)) {
        continue;
      }

      double radian = urg.index2rad(i);
      long x = static_cast<long>(l * cos(radian));
      long y = static_cast<long>(l * sin(radian));

      // x = -1000 ~ 6000 を 0 ~ 800
      // y = -6000 ~ 6000 を 0 ~ 800 へ変換
      int ix = (x - (-1000))/(6000.0 - (-1000.0)) * IMG_SIZE.width;
      int iy = (6000 - y)/(6000.0 - (-6000.0)) * IMG_SIZE.height;
      if (ix >= 0 && ix < IMG_SIZE.width && iy >= 0 && iy < IMG_SIZE.height) {
        cv::circle(img, cv::Point(ix, iy), 1, OBSTACLE_COLOR, -1, cv::LINE_AA);
      }
    }
    cv::imshow("TEST", img);
    cv::waitKey(5);
  }

  return 0;
}
