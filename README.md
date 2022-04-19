# get_distance_with_gnuplot
北陽電機の2D-LiDARライブラリを改造してOpenCVで描画する．

## usage
```
./get_distance_with_opencv
```
デフォルトはUSB接続方式．`-e <IP address>` でイーサネット接続方式へ変更．
いずれの場合も，接続デバイス情報等は`Connection_information.cpp`内で設定する．

