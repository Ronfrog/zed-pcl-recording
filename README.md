# zed-pcl-recording

## 簡介

- 這是使用zed2相機來進行連續錄製點雲的程式，會將點雲儲存成.pcd格式。
- 錄製點雲的同時還可以進行RGB影像的錄製，會儲存成zed sdk使用的.svo格式，如何使用.svo格式檔案請參閱zed sdk的說明。(https://www.stereolabs.com/docs/video/using-video/)
- 錄製同時也會顯示錄到的點雲還有RGB影像，可以視自己需求關閉顯示功能以增加FPS。
- 其中除了zed自己的sdk，還使用了PCL(Point Cloud Library)程式庫。
- 如果想知道其他使用方法，請參照ZED Documentation(https://www.stereolabs.com/docs/) ，還有PCL Documentation(https://pointclouds.org/documentation/) 。

## 系統需求

- [ZED SDK **3**](https://www.stereolabs.com/developers/) 和ZED SDK 3需要的  ([CUDA](https://developer.nvidia.com/cuda-downloads))
- [PCL](https://pointclouds.org/)
- [openCV](https://opencv.org/)

## 建置方法

### windows

- 建立"build"資料夾。
- 使用 cmake-gui 並選擇source和build資料夾。
- 先執行configure，其中選擇自己的Visual Studio版本還有電腦平台。
- Generate 一個 Visual Studio `Win64` solution。
- 開啟剛剛的 solution 並且轉換到 `Release`模式。
- 在Visual Studio設定opencv路徑，一些PCL需要的library可能也需要設定(如flann等)。
- Build solution。

## 如何執行

### windows

- 建置完成以後會在release資料夾底下有個執行檔，直接執行便可使用。
- 開啟後會需要輸入儲存資料的路徑，輸入後面便會開始錄RGB。
- 由於pcd檔案龐大，因此設定為按下'r'鍵後才開始錄製點雲，pcd檔會照frame來編號，因此很容易就可以找到對應的RGB frame。
- 錄影結束後按下'q'結束程式。
