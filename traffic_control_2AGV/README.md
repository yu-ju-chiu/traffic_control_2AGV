# traffic_control_2AGV

#### 環境依賴

VS code v1.57.1
C/C++ms-vscode.cpptools v1.5.0-insiders

#### 部署步驟
1. 以matlab轉換地圖得到`地圖.txt`(此範例為`clear_map.txt`)
2. 當前資料夾開啟terminal
3. `$ make` 以產生traffic_control_2AGV_enter.out
4. `$./traffic_control_2AGV_enter.out`
5. 即有執行檔

#### 演算法流程
1. 空地圖先印出來
3. 將AGV1與AGV2需要變數事先宣告
4. 使用者輸入起點與終點座標點
5. 使用者輸入使用方法(等待:1替代:2)(目前內設為1)
替代:
6. AGV1會先AStar最短路徑並更新地圖(被歸類為路徑於地圖標示為不可行走)
7. AGV2會根據更新的地圖也AStar出路徑
8. 此時兩路徑不會交會
等待:
6. AGV1會先AStar最短路徑
7. AGV2接著AStar出路徑
8. 兩路徑放入array做比較
9. 若兩個路徑相疊，則候車於衝突點前一節點做等待
10. 此時兩AGV不會在同個時間與同個節點交會

```c++
//兩種交管切換 1:路徑衝突時後車等待 2:替代道路
#define METHOD 1
//定義地圖大小
#define X_MAP 42
#define Y_MAP 27
//定義AGV1起始與終點節點
#define START_Y1 15
#define START_X1 18
#define END_Y1 10
#define END_X1 31
//定義AGV2起始與終點節點
#define START_Y2 18
#define START_X2 21
#define END_Y2 12
#define END_X2 33
```

#### V1.0.0 版本內容更新

1. 地圖簡單可視化
2. 使用者可輸入變數
