# RECORD
## 2021.7.6
### 任務目前進度
0. 演算法邏輯-流程圖(7/6)(位置root/檔名:流程圖.png)
2. readme寫入地圖匯入方式(7/6)(位置root/檔名:README.md)
3. 點陣圖轉為矩陣 (7/6)(位置root/檔名:get_map.m)
4. 給使用者可以輸入，起始點，方法(目前為兩台車)
5. 在顯示上，標示出起始點與座標(7/6)(位置root)
6. 多案例*10 會車...(7/7)(位置root)

### 問題
#### 1. 
當地圖上的車道大於一個pixel時，車子會走歪，即便到了終點，也非最短路徑
ex:輸入
5
50
5
46
60
48
60
41
輸出會是這樣的路徑，會發現路徑走歪
![image](https://user-images.githubusercontent.com/38370262/124549787-3f97b400-de62-11eb-8bf9-16052af743ce.png)
#### 1.sol
1. F = G + H原先非使用曼哈頓棋盤法
2. 陣列比大小時有錯誤
3. pointer 指向錯誤
#### 2. 
讀取地圖size時會錯誤，記憶體會爆炸，但在window系統可以使用
#### 2.sol
因為window 與 linux對換行語法不同
window->程式內\r\n->顯示\n
linux->程式內\n->顯示\n
ref: https://www.itread01.com/content/1546857845.html

